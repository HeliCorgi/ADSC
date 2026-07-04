#!/usr/bin/env python3
"""WP10c/WP11 abort forensics -> four committed artifacts under generated/.

Two reports from one deterministic replay engine (spec v5 section 6, WP10c +
WP11; R6/R9):

1. WP10c ARCHIVE (generated/wp10_violation_forensics.{csv,md}): the forensic
   record of the ds-v1 campaign under the LEGACY drift-null abort law
   (mission.cpp compute_safe_abort). WP11 replaced the campaign's screened
   abort with the clearing-abort law, so the committed campaign no longer
   exhibits these violations; the archive regenerates deterministically from
   the pinned seed recipe plus the legacy law, which remains in the codebase
   (R15: legacy pins stay regression-tested -- tests/test_forensic14.cpp pins
   the same 14 cases in C++).

2. WP11 ABORT AUDIT (generated/wp11_abort_audit.{csv,md}): an independent
   replay of the committed (post-WP11) campaign under the clearing-abort law
   (mission.cpp clearing_abort_for), hard-gated on reproducing every
   replay-reachable column of generated/wp5_campaign_runs.csv, reporting the
   keep-out result with its Wilson bound and R14 level tag, abort-stage
   usage, the clearance distribution, and the abort_dv design-variable trade.

Python 3 standard library only (R9). No wall-clock timestamps (R6); output is
LF/ASCII and byte-deterministic.

Replay recipe (WP10c/WP11 completion criterion):
  per-run seed  = SplitMix64(master_seed XOR FNV1a64(catalog_name), run_index)
                  [src/campaign.cpp splitmix64_seed/fnv1a64]
  draw stream   = std::mt19937_64(run_seed) >> 11 -> 53-bit uniform in (0,1],
                  explicit Box-Muller with spare caching
                  [src/campaign.cpp CampRng]
  draw order    = solar(1), sensor_noise(1), sensor_bias(3); then per target:
                  v_close(1); abort branch: rel-pos(3), rel-vel(3);
                  sync branch: tumble-rate(1), axis(3), att(3), act-scale(3),
                  act-misalign(3)   [src/campaign.cpp run_one_mission]
  legacy law    = CW drift-null impulse capped at 2.0 m/s, RK4 coast at
                  dt = 8 s over 1 orbital period [mission.cpp
                  compute_safe_abort; main_campaign.cpp screen overrides]
  clearing law  = WP11 3-stage escalation [mission.cpp clearing_abort_for]:
                  (1) drift-null accepted only if the analytic ellipse clears
                  keep_out + abort_clear_margin_m; (2) bounded-clearing
                  radial reshape (u* = x0^2/g - g/4); (3) two-impulse radial
                  retreat hop; legacy Capped fallback past the dv budget.

Usage: make_forensics.py [repo_root]
"""
import csv
import math
import os
import sys

# ---------------------------------------------------------------------------
# Constants mirrored from the C++ source (provenance in brackets). REPLAY
# MIRRORS of pinned values, not tunables; if any drifts from the C++ source
# the full-population cross-validation below fails hard.
# ---------------------------------------------------------------------------
MASTER_SEED = 0x5AD5C0DECAFE2026    # [campaign.hpp CampaignConfig::master_seed]
DS_ID = "ds-v1"                     # [campaign.hpp CampaignConfig::dispersion_set_id]
SCHEMA = "1.1"                      # [campaign.hpp wp5_schema_version]
N_TARGETS = 6                       # [campaign.hpp targets_per_mission]
KITS_INITIAL = 4                    # [campaign.hpp kits_initial]
DV_BUDGET = 140.0                   # [campaign.hpp dv_budget_m_s]
DV_APPROACH = 8.0                   # [campaign.hpp dv_approach_m_s]
DV_SYNC = 3.0                       # [campaign.hpp dv_sync_m_s]
DV_DEPART = 5.0                     # [campaign.hpp dv_depart_m_s]
DV_ABORT = 4.0                      # [campaign.hpp dv_abort_m_s]
DV_PHASING = 15.0                   # [campaign.hpp dv_phasing_m_s]
NOM_CLOSING = 0.10                  # [campaign.hpp nominal_closing_m_s]
SIG_CLOSING = 0.045                 # [campaign.hpp disp_closing_sigma_m_s]
SIG_REL_POS = 40.0                  # [campaign.hpp disp_rel_pos_m]
SIG_REL_VEL = 0.02                  # [campaign.hpp disp_rel_vel_m_s]
SIG_TUMBLE_FRAC = 0.40              # [campaign.hpp disp_tumble_rate_frac]
SIG_AXIS = 0.30                     # [campaign.hpp disp_tumble_axis_rad]
SIG_ATT = 0.35                      # [campaign.hpp disp_init_att_rad]
SIG_ACT_SCALE = 0.10                # [campaign.hpp disp_actuator_scale]
SIG_ACT_MIS = 0.01                  # [campaign.hpp disp_actuator_misalign_rad]
SIG_SENSOR_NOISE_FRAC = 0.30        # [campaign.hpp disp_sensor_noise_frac]
SIG_SENSOR_BIAS = 5.0e-4            # [campaign.hpp disp_sensor_bias_rad]
NOM_SOLAR = 1.0                     # [campaign.hpp nominal_solar_factor]
SIG_SOLAR_FRAC = 0.50               # [campaign.hpp disp_solar_factor_frac]
MAX_V_REL = 0.15                    # [mission.hpp Config::max_v_rel]
ABORT_DV_CAP = 2.0                  # [mission.hpp Config::abort_dv]
KEEP_OUT_M = 200.0                  # [mission.hpp Config::keep_out_radius_m]
CLEAR_MARGIN_M = 20.0               # [mission.hpp Config::abort_clear_margin_m]
R_CLEAR_M = KEEP_OUT_M + CLEAR_MARGIN_M
STANDOFF_FACTOR = 2.0               # [mission.hpp Config::depart_standoff_factor]
SYNC_RATE_DEG = 2.0                 # [mission.hpp Config::sync_target_rate_deg_s]
COAST_PERIODS = 1.0                 # [main_campaign.cpp campaign override]
COAST_DT_S = 8.0                    # [main_campaign.cpp campaign override]
EARTH_MU = 3.986004418e14           # [relmotion.hpp kEarthMu]
EARTH_RADIUS = 6378137.0            # [relmotion.hpp kEarthRadius]
CATALOG_ALT_KM = {                  # [decay.cpp catalog_A/catalog_B]
    "SL-16 / Zenit-2 second stage": 840.0,
    "SL-8 / Kosmos-3M second stage": 750.0,
}
DEG2RAD = math.pi / 180.0
MASK64 = (1 << 64) - 1
WILSON_Z95 = 1.959963984540054      # [campaign.hpp kWilsonZ95]

# The WP10c forensic-14 pin set: the keep-out-violating (catalog, run_index)
# pairs of the ds-v1 campaign under the LEGACY abort law, as classified at
# WP10c (PR #17) and pinned in tests/test_forensic14.cpp. The archive replay
# below must reproduce EXACTLY this set (R15 regression pin, same status as
# the 19.15 s legacy GNC numbers).
FORENSIC14 = {
    ("SL-16 / Zenit-2 second stage", 65),
    ("SL-16 / Zenit-2 second stage", 81),
    ("SL-16 / Zenit-2 second stage", 86),
    ("SL-16 / Zenit-2 second stage", 112),
    ("SL-16 / Zenit-2 second stage", 340),
    ("SL-16 / Zenit-2 second stage", 460),
    ("SL-16 / Zenit-2 second stage", 490),
    ("SL-8 / Kosmos-3M second stage", 88),
    ("SL-8 / Kosmos-3M second stage", 99),
    ("SL-8 / Kosmos-3M second stage", 160),
    ("SL-8 / Kosmos-3M second stage", 254),
    ("SL-8 / Kosmos-3M second stage", 362),
    ("SL-8 / Kosmos-3M second stage", 383),
    ("SL-8 / Kosmos-3M second stage", 419),
}

# ------------------------------------------------------------------ RNG replay


def fnv1a64(s):
    """FNV-1a 64-bit hash [src/campaign.cpp fnv1a64]."""
    h = 1469598103934665603
    for ch in s.encode("ascii"):
        h ^= ch
        h = (h * 1099511628211) & MASK64
    return h


def splitmix64_seed(master_seed, index):
    """SplitMix64 finalizer [src/campaign.cpp splitmix64_seed]."""
    z = (master_seed + ((index + 1) * 0x9E3779B97F4A7C15)) & MASK64
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return (z ^ (z >> 31)) & MASK64


class MT19937_64:
    """std::mt19937_64 (the C++11-pinned 64-bit Mersenne Twister)."""

    N, M = 312, 156
    MATRIX_A = 0xB5026F5AA96619E9
    UPPER = 0xFFFFFFFF80000000
    LOWER = 0x7FFFFFFF

    def __init__(self, seed):
        mt = [0] * self.N
        mt[0] = seed & MASK64
        for i in range(1, self.N):
            mt[i] = (6364136223846793005 * (mt[i - 1] ^ (mt[i - 1] >> 62)) + i) & MASK64
        self.mt = mt
        self.mti = self.N

    def genrand(self):
        mt, N, M = self.mt, self.N, self.M
        if self.mti >= N:
            for i in range(N - M):
                x = (mt[i] & self.UPPER) | (mt[i + 1] & self.LOWER)
                mt[i] = mt[i + M] ^ (x >> 1) ^ (self.MATRIX_A if x & 1 else 0)
            for i in range(N - M, N - 1):
                x = (mt[i] & self.UPPER) | (mt[i + 1] & self.LOWER)
                mt[i] = mt[i + (M - N)] ^ (x >> 1) ^ (self.MATRIX_A if x & 1 else 0)
            x = (mt[N - 1] & self.UPPER) | (mt[0] & self.LOWER)
            mt[N - 1] = mt[M - 1] ^ (x >> 1) ^ (self.MATRIX_A if x & 1 else 0)
            self.mti = 0
        y = mt[self.mti]
        self.mti += 1
        y ^= (y >> 29) & 0x5555555555555555
        y ^= (y << 17) & 0x71D67FFFEDA60000
        y ^= (y << 37) & 0xFFF7EEE000000000
        y ^= y >> 43
        return y & MASK64


class CampRng:
    """Box-Muller Gaussian source with spare caching [src/campaign.cpp CampRng]."""

    def __init__(self, seed):
        self._gen = MT19937_64(seed)
        self._have_spare = False
        self._spare = 0.0

    def _uniform01(self):
        bits = self._gen.genrand() >> 11
        return (float(bits) + 1.0) * (1.0 / 9007199254740992.0)

    def sample(self):
        if self._have_spare:
            self._have_spare = False
            return self._spare
        u1 = self._uniform01()
        u2 = self._uniform01()
        r = math.sqrt(-2.0 * math.log(u1))
        a = 2.0 * math.pi * u2
        self._spare = r * math.sin(a)
        self._have_spare = True
        return r * math.cos(a)

    def sample3(self, sigma):
        x = self.sample()
        y = self.sample()
        z = self.sample()
        return (x * sigma, y * sigma, z * sigma)


# ------------------------------------------------------------------ CW physics


def cw_derivative(x, n):
    """CW coast derivative [src/relmotion.cpp CwModel::derivative]."""
    return (x[3], x[4], x[5],
            3.0 * n * n * x[0] + 2.0 * n * x[4],
            -2.0 * n * x[3],
            -n * n * x[2])


def cw_rk4_step(x, h, n):
    """One fixed RK4 step [src/relmotion.cpp CwModel::propagate_rk4]."""
    k1 = cw_derivative(x, n)
    k2 = cw_derivative(tuple(x[i] + 0.5 * h * k1[i] for i in range(6)), n)
    k3 = cw_derivative(tuple(x[i] + 0.5 * h * k2[i] for i in range(6)), n)
    k4 = cw_derivative(tuple(x[i] + h * k3[i] for i in range(6)), n)
    return tuple(x[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
                 for i in range(6))


def rel_range6(x):
    return math.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])


def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def bounded_coast_min_range(x0, y0, z0, vx, vz, n):
    """Drift-free coast minimum [src/relmotion.cpp bounded_coast_min_range]."""
    best = float("inf")
    samples = 1440
    for i in range(samples):
        nt = 2.0 * math.pi * float(i) / samples
        c = math.cos(nt)
        s = math.sin(nt)
        x = x0 * c + (vx / n) * s
        y = y0 - 2.0 * x0 * s + 2.0 * (vx / n) * (c - 1.0)
        z = z0 * c + (vz / n) * s
        r = math.sqrt(x * x + y * y + z * z)
        if r < best:
            best = r
    return best


def coast_verify(r, v, dv, n):
    """Single-burn coast sweep [mission.cpp coast_verify_min_range] under the
    campaign screen overrides (1 period at 8 s)."""
    x = (r[0], r[1], r[2], v[0] + dv[0], v[1] + dv[1], v[2] + dv[2])
    mr = rel_range6(x)
    horizon = COAST_PERIODS * (2.0 * math.pi / n)
    t = 0.0
    while t < horizon:
        x = cw_rk4_step(x, COAST_DT_S, n)
        mr = min(mr, rel_range6(x))
        t += COAST_DT_S
    return mr


class AbortOutcome(object):
    __slots__ = ("status", "coast_min_m", "coast_min_t_s", "bounded_min_m",
                 "dv", "dv_mag_uncapped", "dv2_mag")


def legacy_abort(r, v, n):
    """LEGACY drift-null law [mission.cpp compute_safe_abort], with the
    coast-minimum TIME also tracked (for the archive's forensic table)."""
    out = AbortOutcome()
    v_target = (0.0, -2.0 * n * r[0], 0.0)
    dv = tuple(v_target[i] - v[i] for i in range(3))
    mag = norm3(dv)
    capped = mag > ABORT_DV_CAP and mag > 1e-12
    dv_cmd = tuple((ABORT_DV_CAP / mag) * c for c in dv) if capped else dv
    x = (r[0], r[1], r[2], v[0] + dv_cmd[0], v[1] + dv_cmd[1], v[2] + dv_cmd[2])
    mr = rel_range6(x)
    mt = 0.0
    horizon = COAST_PERIODS * (2.0 * math.pi / n)
    t = 0.0
    while t < horizon:
        x = cw_rk4_step(x, COAST_DT_S, n)
        t += COAST_DT_S
        rr = rel_range6(x)
        if rr < mr:
            mr, mt = rr, t
    out.status = "Capped" if capped else "Clean"
    out.coast_min_m = mr
    out.coast_min_t_s = mt
    out.bounded_min_m = (-1.0 if capped else
                         bounded_coast_min_range(r[0], r[1], r[2], 0.0, 0.0, n))
    out.dv = dv_cmd
    out.dv_mag_uncapped = mag
    out.dv2_mag = 0.0
    return out


def clearing_abort(r, v, n, dv_cap=ABORT_DV_CAP):
    """WP11 clearing-abort law [mission.cpp clearing_abort_for], campaign
    screen overrides. dv_cap parameterized for the abort_dv trade sweep."""
    out = AbortOutcome()
    out.coast_min_t_s = 0.0
    x0, y0, z0 = r
    # Stage 1: drift-null baseline.
    v1 = (0.0, -2.0 * n * x0, 0.0)
    dv1 = (v1[0] - v[0], v1[1] - v[1], v1[2] - v[2])
    if norm3(dv1) <= dv_cap:
        m1 = bounded_coast_min_range(x0, y0, z0, 0.0, 0.0, n)
        if m1 >= R_CLEAR_M:
            out.status = "Clean"
            out.dv = dv1
            out.dv_mag_uncapped = norm3(dv1)
            out.bounded_min_m = m1
            out.coast_min_m = coast_verify(r, v, dv1, n)
            out.dv2_mag = 0.0
            return out
    # Stage 2: bounded-clearing radial reshape.
    if abs(y0) > R_CLEAR_M:
        g = abs(y0) - R_CLEAR_M
        u = x0 * x0 / g - 0.25 * g
        if u > 0.0:
            neg_sign_y0 = 1.0 if y0 < 0.0 else -1.0
            vx_star = neg_sign_y0 * n * u
            v2 = (vx_star, -2.0 * n * x0, 0.0)
            dv2 = (v2[0] - v[0], v2[1] - v[1], v2[2] - v[2])
            if norm3(dv2) <= dv_cap:
                m2 = bounded_coast_min_range(x0, y0, z0, vx_star, 0.0, n)
                if m2 >= R_CLEAR_M:
                    out.status = "BoundedClearing"
                    out.dv = dv2
                    out.dv_mag_uncapped = norm3(dv2)
                    out.bounded_min_m = m2
                    out.coast_min_m = coast_verify(r, v, dv2, n)
                    out.dv2_mag = 0.0
                    return out
    # Stage 3: two-impulse radial retreat hop, legacy Capped fallback.
    b = dv1
    b_norm = norm3(b)

    def capped():
        if b_norm > 1e-12:
            dvc = tuple((dv_cap / b_norm) * c for c in b)
        else:
            dvc = (0.0, 0.0, 0.0)
        out.status = "Capped"
        out.dv = dvc
        out.dv_mag_uncapped = b_norm
        out.bounded_min_m = -1.0
        out.coast_min_m = coast_verify(r, v, dvc, n)
        out.dv2_mag = 0.0
        return out

    if b_norm >= dv_cap:
        return capped()
    sigma = 1.0 if y0 <= 0.0 else -1.0
    S = max(0.0, (R_CLEAR_M + 2.0 * abs(x0) + CLEAR_MARGIN_M) - abs(y0))
    v_hop = n * S / 4.0
    v_plus = (sigma * v_hop, -2.0 * n * x0, 0.0)
    dv_b1 = (v_plus[0] - v[0], v_plus[1] - v[1], v_plus[2] - v[2])
    if norm3(dv_b1) + v_hop > dv_cap:
        return capped()
    y_final = y0 - sigma * 4.0 * v_hop / n
    out.status = "RetreatHop"
    out.dv = dv_b1
    out.dv_mag_uncapped = norm3(dv_b1)
    out.bounded_min_m = bounded_coast_min_range(-x0, y_final, -z0, 0.0, 0.0, n)
    x = (r[0], r[1], r[2], v[0] + dv_b1[0], v[1] + dv_b1[1], v[2] + dv_b1[2])
    mr = rel_range6(x)
    horizon = COAST_PERIODS * (2.0 * math.pi / n)
    half_period = 0.5 * (2.0 * math.pi / n)
    burn2_applied = False
    dv2_mag = 0.0
    t = 0.0
    while t < horizon:
        x = cw_rk4_step(x, COAST_DT_S, n)
        t += COAST_DT_S
        mr = min(mr, rel_range6(x))
        if not burn2_applied and t >= half_period:
            vx_b, vz_b = x[3], x[5]
            x = (x[0], x[1], x[2], 0.0, x[4], 0.0)
            dv2_mag = math.sqrt(vx_b * vx_b + vz_b * vz_b)
            burn2_applied = True
            mr = min(mr, rel_range6(x))
    out.coast_min_m = mr
    out.dv2_mag = dv2_mag
    return out


# ------------------------------------------------------------- mission replay


class AbortEvent(object):
    __slots__ = ("target_index", "abort_ordinal", "dv_remaining_before",
                 "v_close", "r", "v", "law")


def replay_run(catalog, run_index, law):
    """Termination-aware replay of run_one_mission under `law` ('legacy' or
    'clearing'). The draw stream is law-independent; only termination and the
    recorded abort outcomes differ. Single modeled assumption (guarded in
    main): every sync attempt succeeds."""
    alt_km = CATALOG_ALT_KM[catalog]
    a = EARTH_RADIUS + alt_km * 1000.0
    n = math.sqrt(EARTH_MU / (a * a * a))
    seed = splitmix64_seed(MASTER_SEED ^ fnv1a64(catalog), run_index)
    rng = CampRng(seed)

    solar = max(0.05, NOM_SOLAR * (1.0 + SIG_SOLAR_FRAC * rng.sample()))
    rng.sample()                       # sensor_noise_scale: drawn, not propagated
    rng.sample3(SIG_SENSOR_BIAS)       # sensor_bias: drawn, not propagated

    dv_rem = DV_BUDGET
    kits = KITS_INITIAL
    dv_used = 0.0
    removals = attempted = aborts = 0
    first_close = None
    tumble_deg = None
    worst_clearance = float("inf")
    outcome = "completed"
    events = []

    for i in range(N_TARGETS):
        if kits == 0:
            outcome = "kit_exhausted"
            break
        if dv_rem < DV_APPROACH:
            outcome = "dv_exhausted"
            break
        attempted += 1
        dv_rem -= DV_APPROACH
        dv_used += DV_APPROACH

        v_close = abs(NOM_CLOSING + SIG_CLOSING * rng.sample())
        if first_close is None:
            first_close = v_close

        if v_close > MAX_V_REL:
            aborts += 1
            if dv_rem < DV_ABORT:
                outcome = "dv_exhausted"
                break
            dv_rem -= DV_ABORT
            dv_used += DV_ABORT
            r_disp = rng.sample3(SIG_REL_POS)
            v_disp = rng.sample3(SIG_REL_VEL)
            r0 = (r_disp[0], -STANDOFF_FACTOR * KEEP_OUT_M + r_disp[1], r_disp[2])
            if law == "legacy":
                ab = legacy_abort(r0, v_disp, n)
            else:
                ab = clearing_abort(r0, v_disp, n)
            ev = AbortEvent()
            ev.target_index = i
            ev.abort_ordinal = aborts
            ev.dv_remaining_before = dv_rem
            ev.v_close = v_close
            ev.r = r0
            ev.v = v_disp
            ev.law = ab
            events.append(ev)
            worst_clearance = min(worst_clearance, ab.coast_min_m - KEEP_OUT_M)
            if ab.coast_min_m < KEEP_OUT_M:
                outcome = "keep_out_violation"
                break
        else:
            if dv_rem < DV_SYNC:
                outcome = "dv_exhausted"
                break
            dv_rem -= DV_SYNC
            dv_used += DV_SYNC
            rate = max(0.05 * DEG2RAD,
                       SYNC_RATE_DEG * DEG2RAD * (1.0 + SIG_TUMBLE_FRAC * rng.sample()))
            rng.sample3(SIG_AXIS)
            rng.sample3(SIG_ATT)
            rng.sample3(SIG_ACT_SCALE)
            rng.sample3(SIG_ACT_MIS)
            if tumble_deg is None:
                tumble_deg = rate / DEG2RAD
            # sync succeeds (guarded assumption)
            if dv_rem < DV_DEPART:
                outcome = "dv_exhausted"
                break
            dv_rem -= DV_DEPART
            dv_used += DV_DEPART
            kits -= 1
            removals += 1

        if i + 1 < N_TARGETS:
            if dv_rem < DV_PHASING:
                outcome = "dv_exhausted"
                break
            dv_rem -= DV_PHASING
            dv_used += DV_PHASING

    return {
        "run_seed": seed,
        "solar": solar,
        "first_close": first_close,
        "tumble_deg": tumble_deg,
        "dv_used": dv_used,
        "dv_rem": dv_rem,
        "removals": removals,
        "attempted": attempted,
        "aborts": aborts,
        "kits_used": KITS_INITIAL - kits,
        "outcome": outcome,
        "worst_clearance": worst_clearance,
        "events": events,
    }


# ------------------------------------------------------------------- checking


def cross_validate(row, rep, check_wp11_columns):
    """Compare a replayed run against its committed CSV row at the CSV's own
    printed precision. Returns mismatch descriptions (empty = pass)."""
    bad = []

    def chk(name, got, want):
        if got != want:
            bad.append("%s: replay %s != csv %s" % (name, got, want))

    chk("run_seed", str(rep["run_seed"]), row["run_seed"])
    chk("solar_factor", "%.5f" % rep["solar"], row["solar_factor"])
    chk("first_closing_speed", "%.5f" % rep["first_close"],
        row["first_closing_speed_m_per_s"])
    if rep["tumble_deg"] is not None:
        chk("tumble_rate", "%.5f" % rep["tumble_deg"], row["tumble_rate_deg_per_s"])
    chk("dv_used", "%.4f" % rep["dv_used"], row["dv_used_m_per_s"])
    chk("dv_remaining", "%.4f" % rep["dv_rem"], row["dv_remaining_m_per_s"])
    chk("removals", str(rep["removals"]), row["removals"])
    chk("targets_attempted", str(rep["attempted"]), row["targets_attempted"])
    chk("gate_abort_events", str(rep["aborts"]), row["gate_abort_events"])
    chk("kits_used", str(rep["kits_used"]), row["kits_used"])
    chk("outcome", rep["outcome"], row["outcome"])
    if check_wp11_columns:
        chk("schema_version", SCHEMA, row["schema_version"])
        chk("dispersion_set_id", DS_ID, row["dispersion_set_id"])
        want_clear = ("%.4f" % rep["worst_clearance"]) if rep["aborts"] > 0 else ""
        chk("worst_abort_clearance", want_clear, row["worst_abort_clearance_m"])
    return bad


def wilson_interval(k, nn, z=WILSON_Z95):
    """[campaign.cpp wilson_interval]"""
    if nn <= 0:
        return (0.0, 0.0)
    phat = float(k) / float(nn)
    z2 = z * z
    denom = 1.0 + z2 / nn
    center = (phat + z2 / (2.0 * nn)) / denom
    margin = (z / denom) * math.sqrt(phat * (1.0 - phat) / nn + z2 / (4.0 * nn * nn))
    return (max(0.0, center - margin), min(1.0, center + margin))


def f(x, nd):
    return ("%." + str(nd) + "f") % x


def ellipse_min_range(x0, y0, z0):
    """Clean-abort drift-free ellipse minimum (legacy law post-burn state)."""
    return bounded_coast_min_range(x0, y0, z0, 0.0, 0.0, 1.0e-3)


# ----------------------------------------------------------------------- main


def classify_legacy(ev):
    initial_range = norm3(ev.r)
    if initial_range < KEEP_OUT_M:
        return "starts_inside_keep_out"
    if ev.law.status == "Capped":
        return "capped_abort_residual_drift"
    return "clean_abort_safety_ellipse_intersects_keep_out"


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..")
    root = os.path.abspath(root)
    runs_path = os.path.join(root, "generated", "wp5_campaign_runs.csv")
    wp10_csv = os.path.join(root, "generated", "wp10_violation_forensics.csv")
    wp10_md = os.path.join(root, "generated", "wp10_violation_forensics.md")
    wp11_csv = os.path.join(root, "generated", "wp11_abort_audit.csv")
    wp11_md = os.path.join(root, "generated", "wp11_abort_audit.md")

    with open(runs_path, newline="") as fh:
        rows = list(csv.DictReader(fh))

    total_sync_timeouts = sum(int(r["sync_timeout_events"]) for r in rows)
    if total_sync_timeouts != 0:
        raise SystemExit(
            "wp5_campaign_runs.csv has %d sync_timeout_events; the replay's "
            "sync-success assumption does not hold. Extend the tool before "
            "classifying." % total_sync_timeouts)
    unknown = sorted({r["catalog"] for r in rows} - set(CATALOG_ALT_KM))
    if unknown:
        raise SystemExit("unknown catalog preset(s) %s; extend CATALOG_ALT_KM "
                         "from decay.cpp before classifying." % unknown)

    n_runs = len(rows)

    # ---------------- PASS A: WP11 audit (clearing law vs committed CSV).
    mismatches = 0
    audit_events = []          # (catalog, run_index, run_seed, AbortEvent)
    committed_violations = 0
    per_cat = {}
    for row in rows:
        cat, idx = row["catalog"], int(row["run_index"])
        per_cat.setdefault(cat, []).append(row)
        rep = replay_run(cat, idx, "clearing")
        bad = cross_validate(row, rep, check_wp11_columns=True)
        if bad:
            mismatches += 1
            if mismatches <= 5:
                sys.stderr.write("wp11 audit mismatch %s/%d: %s\n"
                                 % (cat, idx, "; ".join(bad)))
        if row["keep_out_violation"] == "true":
            committed_violations += 1
        for ev in rep["events"]:
            audit_events.append((cat, idx, rep["run_seed"], ev))
    if mismatches or committed_violations:
        raise SystemExit(
            "WP11 audit replay FAILED (%d/%d mismatches; %d committed "
            "violations -- D13 requires 0). The replay mirrors are stale or "
            "the campaign regressed; fix before writing artifacts."
            % (mismatches, n_runs, committed_violations))

    # ---------------- PASS B: WP10 archive (legacy law from the same seeds).
    legacy_violations = []     # (catalog, run_index, rep, terminal AbortEvent)
    legacy_stream_mism = 0
    for row in rows:
        cat, idx = row["catalog"], int(row["run_index"])
        rep = replay_run(cat, idx, "legacy")
        # Stream-level columns are law-independent: they must match the
        # committed (post-WP11) CSV for every run.
        for name, got, want in (
                ("run_seed", str(rep["run_seed"]), row["run_seed"]),
                ("solar", "%.5f" % rep["solar"], row["solar_factor"]),
                ("first_close", "%.5f" % rep["first_close"],
                 row["first_closing_speed_m_per_s"])):
            if got != want:
                legacy_stream_mism += 1
                sys.stderr.write("wp10 archive stream mismatch %s/%d %s\n"
                                 % (cat, idx, name))
        if rep["outcome"] == "keep_out_violation":
            legacy_violations.append((cat, idx, rep, rep["events"][-1]))
    legacy_set = {(c, i) for (c, i, _, _) in legacy_violations}
    if legacy_stream_mism or legacy_set != FORENSIC14:
        raise SystemExit(
            "WP10 archive replay FAILED (stream mismatches: %d; legacy "
            "violation set %s the pinned forensic-14). The legacy law or the "
            "seed recipe drifted; fix before writing artifacts."
            % (legacy_stream_mism,
               "==" if legacy_set == FORENSIC14 else "!="))

    # ---------------- WP10 archive CSV (byte-compatible with the WP10c one).
    fieldnames = [
        "schema_version", "catalog", "run_index", "run_seed", "target_index",
        "gate_abort_ordinal", "closing_speed_m_per_s",
        "dv_remaining_at_abort_m_per_s", "r0_x_m", "r0_y_m", "r0_z_m",
        "v0_x_m_per_s", "v0_y_m_per_s", "v0_z_m_per_s", "initial_range_m",
        "corridor_deviation_m", "abort_dv_x_m_per_s", "abort_dv_y_m_per_s",
        "abort_dv_z_m_per_s", "abort_dv_mag_uncapped_m_per_s",
        "abort_dv_cap_m_per_s", "capped", "post_burn_drift_m_per_s",
        "coast_min_range_m", "coast_min_time_s",
        "analytic_ellipse_min_range_m", "keep_out_radius_m", "classification",
    ]
    out_rows = []
    for (cat, idx, rep, ev) in legacy_violations:
        x0, y0, z0 = ev.r
        n = math.sqrt(EARTH_MU / ((EARTH_RADIUS + CATALOG_ALT_KM[cat] * 1000.0) ** 3))
        vpost_y = ev.v[1] + ev.law.dv[1]
        drift = abs(vpost_y + 2.0 * n * x0)
        corridor_dev = math.sqrt(ev.r[0] ** 2 +
                                 (ev.r[1] + STANDOFF_FACTOR * KEEP_OUT_M) ** 2 +
                                 ev.r[2] ** 2)
        out_rows.append({
            "schema_version": "1.0",
            "catalog": cat,
            "run_index": str(idx),
            "run_seed": str(rep["run_seed"]),
            "target_index": str(ev.target_index),
            "gate_abort_ordinal": str(ev.abort_ordinal),
            "closing_speed_m_per_s": f(ev.v_close, 5),
            "dv_remaining_at_abort_m_per_s": f(ev.dv_remaining_before, 4),
            "r0_x_m": f(x0, 4), "r0_y_m": f(y0, 4), "r0_z_m": f(z0, 4),
            "v0_x_m_per_s": f(ev.v[0], 6),
            "v0_y_m_per_s": f(ev.v[1], 6),
            "v0_z_m_per_s": f(ev.v[2], 6),
            "initial_range_m": f(math.sqrt(x0 ** 2 + y0 ** 2 + z0 ** 2), 4),
            "corridor_deviation_m": f(corridor_dev, 4),
            "abort_dv_x_m_per_s": f(ev.law.dv[0], 6),
            "abort_dv_y_m_per_s": f(ev.law.dv[1], 6),
            "abort_dv_z_m_per_s": f(ev.law.dv[2], 6),
            "abort_dv_mag_uncapped_m_per_s": f(ev.law.dv_mag_uncapped, 6),
            "abort_dv_cap_m_per_s": f(ABORT_DV_CAP, 1),
            "capped": "true" if ev.law.status == "Capped" else "false",
            "post_burn_drift_m_per_s": f(drift, 9),
            "coast_min_range_m": f(ev.law.coast_min_m, 4),
            "coast_min_time_s": f(ev.law.coast_min_t_s, 1),
            "analytic_ellipse_min_range_m":
                f(bounded_coast_min_range(x0, y0, z0, 0.0, 0.0, n), 4),
            "keep_out_radius_m": f(KEEP_OUT_M, 1),
            "classification": classify_legacy(ev),
        })
    with open(wp10_csv, "w", newline="\n") as fh:
        wr = csv.DictWriter(fh, fieldnames=fieldnames, lineterminator="\n")
        wr.writeheader()
        for r in out_rows:
            wr.writerow(r)

    # ---------------- WP10 archive md.
    n_capped = sum(1 for r in out_rows if r["capped"] == "true")
    n_clean = len(out_rows) - n_capped
    per_catalog_counts = {}
    for r in out_rows:
        per_catalog_counts[r["catalog"]] = per_catalog_counts.get(r["catalog"], 0) + 1
    lines = []
    w = lines.append
    w("# WP10c keep-out violation forensics (ARCHIVE: legacy abort law, ds-v1)")
    w("")
    w("**Supersession note (WP11).** This is the WP10c forensic record of the")
    w("ds-v1 campaign under the LEGACY drift-null abort law")
    w("(`compute_safe_abort`). WP11 replaced the campaign's screened abort with")
    w("the clearing-abort law (`compute_clearing_abort`); the committed")
    w("campaign CSV no longer exhibits these violations (see")
    w("`wp11_abort_audit.md` for the current keep-out result and the R15")
    w("changelog in the evidence pack for the archived rates). This archive")
    w("regenerates deterministically from the pinned seed recipe plus the")
    w("legacy law, which remains in the codebase; the same 14 cases are pinned")
    w("in C++ in `tests/test_forensic14.cpp` (R15).")
    w("")
    w("## 1. Replay recipe")
    w("")
    w("- Per-run seed: `SplitMix64(master_seed XOR FNV1a64(catalog), run_index)`")
    w("  with `master_seed = 0x5AD5C0DECAFE2026` (campaign.hpp); the run seeds in")
    w("  the forensics table below equal the committed `run_seed` column.")
    w("- Draw stream: `std::mt19937_64(run_seed)`, 53-bit uniforms in (0, 1],")
    w("  explicit Box-Muller with spare caching, in the exact draw order of")
    w("  `run_one_mission` (campaign.cpp).")
    w("- Abort physics: LEGACY Clohessy-Wiltshire drift-null impulse capped at")
    w("  2.0 m/s (mission.cpp `compute_safe_abort`), RK4 coast at dt = 8 s over")
    w("  1 orbital period of the catalog altitude (the campaign's coarsened")
    w("  screen, main_campaign.cpp), keep-out radius 200 m, standoff 400 m.")
    w("- To reproduce: `python3 tools/forensics/make_forensics.py` from the repo")
    w("  root, or `bash tools/regenerate_all.sh build` for the full pipeline.")
    w("")
    w("## 2. Replay cross-validation (gate for everything below)")
    w("")
    w("- Stream-level columns (run_seed, solar_factor, first_closing_speed) of")
    w("  the legacy replay match the committed schema-1.1 CSV for all %d runs" % n_runs)
    w("  (these draws are abort-law-independent).")
    w("- Replayed under the LEGACY law, exactly %d runs terminate in" % len(out_rows))
    w("  keep_out_violation (%s), matching the WP10c"
      % ", ".join("%s: %d" % (k, per_catalog_counts[k])
                  for k in sorted(per_catalog_counts)))
    w("  classification and the C++ forensic-14 pin set; every other run's full")
    w("  bookkeeping is identical under both laws (their aborts clear either")
    w("  way). The clearing-law replay of the committed campaign is separately")
    w("  hard-gated in `wp11_abort_audit.md` section 2.")
    w("- Modeled assumption, guarded: every sync attempt succeeds (the committed")
    w("  CSV contains 0 sync_timeout_events across all %d runs)." % n_runs)
    w("")
    w("## 3. Findings (WP10c, archived)")
    w("")
    max_dv_mag = max(float(r["abort_dv_mag_uncapped_m_per_s"]) for r in out_rows)
    w("**The D13 working hypothesis is REFUTED by the data.** D13 hypothesized the")
    w("primary mechanism as *capped-abort residual drift* re-entering the keep-out")
    w("sphere. In fact **all %d violations are CLEAN aborts**: the commanded" % n_clean)
    w("drift-null impulse (max %s m/s across the violating runs) never reaches"
      % f(max_dv_mag, 3))
    w("the 2.0 m/s cap, and the post-burn secular drift term |vy + 2 n x| is")
    w("numerically zero for every violating run (column `post_burn_drift_m_per_s`).")
    w("Capped aborts observed among the violations: %d." % n_capped)
    w("")
    w("**Actual mechanism -- clean-abort safety-ellipse keep-out intersection.**")
    w("The legacy abort law nulls the drift, which yields a *bounded* relative")
    w("orbit, not necessarily a *clearing* one. The post-burn coast is the")
    w("closed-form ellipse `x(t) = x0 cos(nt)`, `y(t) = y0 - 2 x0 sin(nt)`,")
    w("`z(t) = z0 cos(nt)`: a dispersed radial offset x0 produces an along-track")
    w("swing of 2|x0| about the dispersed standoff y0, so the coast approaches")
    w("the target to roughly `|y0| - 2|x0|` (exactly: the sampled ellipse")
    w("minimum, column `analytic_ellipse_min_range_m`, which matches the RK4")
    w("sweep within the 8 s screen resolution). Every violating run has |x0|")
    x0_lo = min(abs(float(r["r0_x_m"])) for r in out_rows)
    x0_hi = max(abs(float(r["r0_x_m"])) for r in out_rows)
    w("between %s m and %s m (%s-%s sigma of the %s m per-axis dispersion),"
      % (f(x0_lo, 1), f(x0_hi, 1), f(x0_lo / SIG_REL_POS, 1),
         f(x0_hi / SIG_REL_POS, 1), f(SIG_REL_POS, 0)))
    w("and the coast minimum occurs at a quarter orbit (x0 < 0) or")
    w("three-quarter orbit (x0 > 0), exactly as the ellipse geometry predicts.")
    w("The violation is a *geometric* property of where the abort starts, not a")
    w("thruster-authority (cap) property. **WP11 addressed exactly this")
    w("mechanism** with the clearing-abort law's analytic-clearance acceptance")
    w("(named design element per the WP11 completion criteria).")
    w("")
    cls_counts = {}
    for r in out_rows:
        cls_counts[r["classification"]] = cls_counts.get(r["classification"], 0) + 1
    w("Classification of all %d violating runs:" % len(out_rows))
    for name in sorted(cls_counts):
        w("`%s`: %d of %d." % (name, cls_counts[name], len(out_rows)))
    for name in ("capped_abort_residual_drift", "starts_inside_keep_out",
                 "clean_abort_safety_ellipse_intersects_keep_out"):
        if name not in cls_counts:
            w("`%s`: 0." % name)
    w("")
    w("Model scope: this analysis is [L0: linear CW coast, campaign keep-out")
    w("screen at dt = 8 s over 1 period, dispersion set ds-v1]; it inherits")
    w("every WP5 model limitation (sensor dispersions drawn but not")
    w("re-propagated; flat placeholder Delta-v costs; see the evidence-pack")
    w("inventory).")
    w("")
    w("## 4. Per-run forensic table")
    w("")
    w("Full precision in `wp10_violation_forensics.csv`; abridged view:")
    w("")
    w("| catalog | run | tgt | x0 [m] | y0 [m] | z0 [m] | dv mag [m/s] | capped |"
      " coast min [m] | t_min [s] | ellipse min [m] | classification |")
    w("|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---|")
    for r in out_rows:
        w("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["catalog"].split(" / ")[0], r["run_index"], r["target_index"],
            f(float(r["r0_x_m"]), 1), f(float(r["r0_y_m"]), 1),
            f(float(r["r0_z_m"]), 1), f(float(r["abort_dv_mag_uncapped_m_per_s"]), 3),
            r["capped"], f(float(r["coast_min_range_m"]), 1),
            f(float(r["coast_min_time_s"]), 0),
            f(float(r["analytic_ellipse_min_range_m"]), 1),
            r["classification"]))
    w("")
    w("## 5. Honest limits")
    w("")
    w("- Linear CW only (L0); no J2/drag/SRP. The WP12 fidelity ladder re-issues")
    w("  safety statements per level.")
    w("- The 8 s screen step under-resolves the exact minimum by up to a few")
    w("  meters versus the analytic ellipse; the classification is unaffected.")
    w("- This archive CLASSIFIES the superseded legacy-law campaign; the")
    w("  committed campaign numbers are those of the WP11 clearing law (see")
    w("  wp5_campaign_summary.md and wp11_abort_audit.md).")
    w("")
    with open(wp10_md, "w", newline="\n") as fh:
        fh.write("\n".join(lines))

    # ---------------- WP11 audit CSV (one row per abort event, clearing law).
    audit_fields = [
        "schema_version", "catalog", "run_index", "run_seed", "target_index",
        "abort_ordinal", "status", "dv_mag_m_per_s", "dv_second_burn_m_per_s",
        "coast_min_range_m", "bounded_min_range_m", "clearance_m",
        "keep_out_radius_m", "clear_margin_m",
    ]
    with open(wp11_csv, "w", newline="\n") as fh:
        wr = csv.DictWriter(fh, fieldnames=audit_fields, lineterminator="\n")
        wr.writeheader()
        for (cat, idx, seed, ev) in audit_events:
            wr.writerow({
                "schema_version": SCHEMA,
                "catalog": cat,
                "run_index": str(idx),
                "run_seed": str(seed),
                "target_index": str(ev.target_index),
                "abort_ordinal": str(ev.abort_ordinal),
                "status": ev.law.status,
                "dv_mag_m_per_s": f(norm3(ev.law.dv), 6),
                "dv_second_burn_m_per_s": f(ev.law.dv2_mag, 6),
                "coast_min_range_m": f(ev.law.coast_min_m, 4),
                "bounded_min_range_m": f(ev.law.bounded_min_m, 4),
                "clearance_m": f(ev.law.coast_min_m - KEEP_OUT_M, 4),
                "keep_out_radius_m": f(KEEP_OUT_M, 1),
                "clear_margin_m": f(CLEAR_MARGIN_M, 1),
            })

    # ---------------- WP11 audit md.
    lines = []
    w = lines.append
    w("# WP11 abort audit (clearing-abort law, committed campaign)")
    w("")
    w("Independent stdlib-Python replay of the committed ds-v1 campaign under")
    w("the WP11 clearing-abort law, hard-gated on reproducing the committed")
    w("`generated/wp5_campaign_runs.csv` before reporting anything. Produced by")
    w("`tools/forensics/make_forensics.py`; regenerated by")
    w("`tools/regenerate_all.sh`. See `wp10_violation_forensics.md` for the")
    w("archived legacy-law forensics that motivated WP11 (D13).")
    w("")
    w("## 1. Replay cross-validation (gate for everything below)")
    w("")
    w("- %d/%d committed runs replayed under the clearing law with ZERO" % (n_runs, n_runs))
    w("  mismatches across every replay-reachable column (run_seed,")
    w("  solar_factor, first_closing_speed, tumble_rate, dv_used, dv_remaining,")
    w("  removals, targets_attempted, gate_abort_events, kits_used, outcome,")
    w("  schema_version, dispersion_set_id, worst_abort_clearance_m), each at")
    w("  the CSV's own printed precision.")
    w("- Columns NOT replayable in Python (attitude-dynamics outputs:")
    w("  sync_time_s, mission_time_s) are not used by this audit.")
    w("- Modeled assumption, guarded: every sync attempt succeeds (0")
    w("  sync_timeout_events committed).")
    w("")
    w("## 2. Keep-out result (WP11 completion criterion, R14-tagged)")
    w("")
    for cat in sorted(per_cat):
        sub = per_cat[cat]
        k = sum(1 for r in sub if r["keep_out_violation"] == "true")
        lo, hi = wilson_interval(k, len(sub))
        w("- %s: keep-out violations **%d of %d** [L0: linear CW, dispersion"
          % (cat, k, len(sub)))
        w("  set ds-v1, Wilson 95%% upper bound %s]." % f(hi, 4))
    w("")
    w("A zero still carries an interval (R15/WP11): the Wilson upper bound is")
    w("quoted above, never a bare zero. The legacy-law campaign at the same")
    w("seeds violated in 14 runs (see the archive); the clearing law changed")
    w("the abort, not the dispersions, so the comparison is seed-for-seed.")
    w("")
    w("## 3. Abort-stage usage (all gate-abort events, both catalogs)")
    w("")
    stage_counts = {}
    for (_, _, _, ev) in audit_events:
        stage_counts[ev.law.status] = stage_counts.get(ev.law.status, 0) + 1
    w("| stage | events |")
    w("|---|---:|")
    for name in ("Clean", "BoundedClearing", "RetreatHop", "Capped"):
        w("| %s | %d |" % (name, stage_counts.get(name, 0)))
    w("| total | %d |" % len(audit_events))
    w("")
    w("Reading: `Clean` = the legacy drift-null impulse already clears keep-out")
    w("+ margin analytically (the majority); `BoundedClearing` = the WP11")
    w("radial reshape was needed (these contain every case the legacy law")
    w("violated); `RetreatHop`/`Capped` = escalations, not expected from the")
    w("campaign's standoff geometry.")
    w("")
    w("## 4. Clearance distribution (coast-verified min range - keep-out)")
    w("")
    clears = sorted(ev.law.coast_min_m - KEEP_OUT_M for (_, _, _, ev) in audit_events)

    def pctl(p):
        if not clears:
            return 0.0
        if len(clears) == 1:
            return clears[0]
        rank = (p / 100.0) * float(len(clears) - 1)
        frac, lo_f = math.modf(rank)
        lo = int(lo_f)
        if lo + 1 >= len(clears):
            return clears[-1]
        return clears[lo] + frac * (clears[lo + 1] - clears[lo])

    w("| statistic | value [m] |")
    w("|---|---:|")
    w("| floor (worst abort event) | %s |" % f(clears[0], 4))
    w("| p05 | %s |" % f(pctl(5.0), 2))
    w("| p50 | %s |" % f(pctl(50.0), 2))
    w("| p95 | %s |" % f(pctl(95.0), 2))
    w("| events | %d |" % len(clears))
    w("")
    w("Under the legacy law the same distribution had a floor of ~-146.7 m (a")
    w("violation, run SL-8/254) and near-misses continuous down to ~10.8 m")
    w("(WP10c archive). The clearing law's analytic acceptance moves the floor")
    w("above the design margin by construction.")
    w("")
    w("## 5. abort_dv as a design variable (WP11 trade, spec v5 section 6)")
    w("")
    w("The same dispersed abort states, swept over the impulse budget cap:")
    w("")
    w("| abort_dv cap [m/s] | Clean | BoundedClearing | RetreatHop | Capped |"
      " violations | clearance floor [m] |")
    w("|---:|---:|---:|---:|---:|---:|---:|")
    for cap in (0.5, 1.0, 2.0, 4.0):
        counts = {}
        floor = float("inf")
        viol = 0
        for (cat, idx, _, ev) in audit_events:
            n = math.sqrt(EARTH_MU / ((EARTH_RADIUS +
                          CATALOG_ALT_KM[cat] * 1000.0) ** 3))
            ab = clearing_abort(ev.r, ev.v, n, dv_cap=cap)
            counts[ab.status] = counts.get(ab.status, 0) + 1
            floor = min(floor, ab.coast_min_m - KEEP_OUT_M)
            if ab.coast_min_m < KEEP_OUT_M:
                viol += 1
        w("| %s | %d | %d | %d | %d | %d | %s |" % (
            f(cap, 1), counts.get("Clean", 0), counts.get("BoundedClearing", 0),
            counts.get("RetreatHop", 0), counts.get("Capped", 0), viol,
            f(floor, 4)))
    w("")
    w("Reading: thruster authority was never the binding constraint (the WP10c")
    w("finding); the trade shows the clearing law is insensitive to the cap")
    w("across 0.5-4.0 m/s for this dispersion set because the reshape and hop")
    w("impulses are small (<~0.4 m/s). `abort_dv` therefore stays at its")
    w("legacy 2.0 m/s value; no pinned number moves on account of this trade.")
    w("")
    w("## 6. Replay recipe and honest limits")
    w("")
    w("- Recipe: identical to the WP10c archive (section 1 there), with the")
    w("  clearing-abort law (mission.cpp `clearing_abort_for`) in place of the")
    w("  legacy law; stage math mirrored per the provenance comments in")
    w("  `tools/forensics/make_forensics.py`.")
    w("- [L0: linear CW, dispersion set ds-v1]; sensor dispersions drawn but")
    w("  not re-propagated (WP5 documented simplification); flat placeholder")
    w("  Delta-v costs; the campaign keep-out screen runs at dt = 8 s over 1")
    w("  period (main_campaign.cpp overrides).")
    w("- This audit reports the committed campaign; it changes no behavior and")
    w("  quotes no number that does not regenerate from the committed CSVs.")
    w("")
    with open(wp11_md, "w", newline="\n") as fh:
        fh.write("\n".join(lines))

    sys.stdout.write(
        "[WP10c/WP11] wrote %s (%d rows), %s, %s (%d rows), %s (%d lines)\n"
        % (os.path.relpath(wp10_csv, root), len(out_rows),
           os.path.relpath(wp10_md, root),
           os.path.relpath(wp11_csv, root), len(audit_events),
           os.path.relpath(wp11_md, root), len(lines)))


if __name__ == "__main__":
    main()
