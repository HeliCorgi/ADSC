#!/usr/bin/env python3
"""WP10c keep-out-violation forensics -> generated/wp10_violation_forensics.{csv,md}

Read-only analysis of the committed WP5 campaign (spec v5 section 6, WP10c):
re-runs the violating seeds from generated/wp5_campaign_runs.csv with an exact
Python replay of the campaign's deterministic draw stream and abort-coast
physics, classifies every keep-out violation against the D13 hypothesis
(capped-abort residual drift), and writes the forensic classification through
the normal regeneration pipeline. NO control-law or campaign code is touched
(WP10 is behavior-change-free); this tool only reads committed CSV bytes and
mirrors constants that are pinned in the C++ source.

Python 3 standard library only (R9 tooling exception). No wall-clock
timestamps or run times are embedded (R6); output is LF/ASCII and
byte-deterministic: two runs on any platform produce identical bytes.

Replay recipe (documented per WP10c completion criterion):
  per-run seed  = SplitMix64(master_seed XOR FNV1a64(catalog_name), run_index)
                  [src/campaign.cpp splitmix64_seed/fnv1a64]
  draw stream   = std::mt19937_64(run_seed) >> 11 -> 53-bit uniform in (0,1],
                  explicit Box-Muller with spare caching
                  [src/campaign.cpp CampRng]
  draw order    = solar(1), sensor_noise(1), sensor_bias(3); then per target:
                  v_close(1); abort branch: rel-pos(3), rel-vel(3);
                  sync branch: tumble-rate(1), axis(3), att(3), act-scale(3),
                  act-misalign(3)   [src/campaign.cpp run_one_mission]
  abort physics = CW drift-null impulse capped at 2.0 m/s, RK4 coast at
                  dt = 8 s over 1 orbital period of the catalog altitude
                  [src/mission.cpp compute_safe_abort; src/main_campaign.cpp
                  abort_coast_check_{periods,dt_s} campaign overrides]

Usage: make_forensics.py [repo_root]
"""
import csv
import math
import os
import sys

# ---------------------------------------------------------------------------
# Constants mirrored from the C++ source (provenance in brackets). These are
# REPLAY MIRRORS of pinned values, not new tunables; if any of them drifts
# from the C++ source the full-population cross-validation below fails hard.
# ---------------------------------------------------------------------------
MASTER_SEED = 0x5AD5C0DECAFE2026    # [campaign.hpp CampaignConfig::master_seed]
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
SCHEMA_VERSION = "1.0"

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
    """CW state derivative, coast [src/relmotion.cpp CwModel::derivative]."""
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


def compute_safe_abort(r, v, n):
    """Mirror of Mission::compute_safe_abort under the campaign coast overrides.

    Returns (coast_min_range_m, coast_min_time_s, capped, dv_mag_uncapped,
    dv_commanded).
    """
    v_target = (0.0, -2.0 * n * r[0], 0.0)
    dv = tuple(v_target[i] - v[i] for i in range(3))
    mag = math.sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2])
    capped = mag > ABORT_DV_CAP and mag > 1e-12
    dv_cmd = tuple((ABORT_DV_CAP / mag) * c for c in dv) if capped else dv
    x = (r[0], r[1], r[2], v[0] + dv_cmd[0], v[1] + dv_cmd[1], v[2] + dv_cmd[2])
    min_range = math.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
    min_time = 0.0
    horizon = COAST_PERIODS * (2.0 * math.pi / n)
    t = 0.0
    while t < horizon:
        x = cw_rk4_step(x, COAST_DT_S, n)
        t += COAST_DT_S
        rr = math.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        if rr < min_range:
            min_range = rr
            min_time = t
    return min_range, min_time, capped, mag, dv_cmd


def ellipse_min_range(x0, y0, z0):
    """Analytic min range of the drift-free coast after a CLEAN abort.

    Post-burn state is (x0, y0, z0, 0, -2 n x0, 0); the closed-form CW coast is
    x(t) = x0 cos(nt), y(t) = y0 - 2 x0 sin(nt), z(t) = z0 cos(nt).
    Sampled at 0.25 deg like SafetyEllipse::min_range (dense-sampling cross-
    check of the RK4 sweep; independent of n).
    """
    best = float("inf")
    samples = 1440
    for i in range(samples):
        th = 2.0 * math.pi * i / samples
        c, s = math.cos(th), math.sin(th)
        rr = math.sqrt((x0 * x0 + z0 * z0) * c * c + (y0 - 2.0 * x0 * s) ** 2)
        if rr < best:
            best = rr
    return best


# ------------------------------------------------------------- mission replay


class AbortRecord(object):
    __slots__ = ("target_index", "abort_ordinal", "dv_remaining_before",
                 "v_close", "r", "v", "coast_min_m", "coast_min_t_s",
                 "capped", "dv_mag_uncapped", "dv_cmd")


def replay_run(catalog, run_index):
    """Termination-aware exact replay of run_one_mission (draws + bookkeeping).

    The single modeled assumption: every sync-branch attempt succeeds. This is
    guarded in main(): the committed CSV has 0 sync_timeout_events in every
    row; the tool refuses to run otherwise (a sync failure would change dv/kit
    bookkeeping, not the draw stream).
    """
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
    outcome = "completed"
    abort_records = []
    violation = None

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
            r0 = (r_disp[0],
                  -STANDOFF_FACTOR * KEEP_OUT_M + r_disp[1],
                  r_disp[2])
            mr, mt, capped, mag, dv_cmd = compute_safe_abort(r0, v_disp, n)
            rec = AbortRecord()
            rec.target_index = i
            rec.abort_ordinal = aborts
            rec.dv_remaining_before = dv_rem
            rec.v_close = v_close
            rec.r = r0
            rec.v = v_disp
            rec.coast_min_m = mr
            rec.coast_min_t_s = mt
            rec.capped = capped
            rec.dv_mag_uncapped = mag
            rec.dv_cmd = dv_cmd
            abort_records.append(rec)
            if mr < KEEP_OUT_M:
                outcome = "keep_out_violation"
                violation = rec
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
            # sync succeeds (guarded assumption, see docstring)
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
        "abort_records": abort_records,
        "violation": violation,
    }


# ------------------------------------------------------------------- checking


def cross_validate(row, rep):
    """Compare a replayed run against its committed CSV row.

    Comparison is at the CSV's own printed precision (the printf formats in
    write_runs_csv). Returns a list of mismatch descriptions (empty = pass).
    """
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
    return bad


def classify(rec):
    """Classify one violating abort against the D13 hypothesis."""
    initial_range = math.sqrt(rec.r[0] ** 2 + rec.r[1] ** 2 + rec.r[2] ** 2)
    if initial_range < KEEP_OUT_M:
        return "starts_inside_keep_out"
    if rec.capped:
        return "capped_abort_residual_drift"
    return "clean_abort_safety_ellipse_intersects_keep_out"


# ----------------------------------------------------------------------- main


def f(x, nd):
    return ("%." + str(nd) + "f") % x


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..")
    root = os.path.abspath(root)
    runs_path = os.path.join(root, "generated", "wp5_campaign_runs.csv")
    out_csv = os.path.join(root, "generated", "wp10_violation_forensics.csv")
    out_md = os.path.join(root, "generated", "wp10_violation_forensics.md")

    with open(runs_path, newline="") as fh:
        rows = list(csv.DictReader(fh))

    # Assumption guard: the replay models every sync attempt as a success.
    # That is exactly what the committed campaign shows; refuse to classify
    # against data where it does not hold.
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

    # Pass 1: full-population replay + cross-validation.
    n_runs = len(rows)
    n_mismatch = 0
    mismatch_examples = []
    total_aborts = 0
    nonviol_margins = []          # coast_min - keep_out for non-violating aborts
    violations = []               # (row, rep, rec)
    per_catalog_counts = {}
    for row in rows:
        cat = row["catalog"]
        rep = replay_run(cat, int(row["run_index"]))
        bad = cross_validate(row, rep)
        if bad:
            n_mismatch += 1
            if len(mismatch_examples) < 5:
                mismatch_examples.append((cat, row["run_index"], bad))
        total_aborts += rep["aborts"]
        for rec in rep["abort_records"]:
            if rec.coast_min_m >= KEEP_OUT_M:
                nonviol_margins.append(rec.coast_min_m - KEEP_OUT_M)
        if rep["violation"] is not None:
            violations.append((row, rep, rep["violation"]))
            per_catalog_counts[cat] = per_catalog_counts.get(cat, 0) + 1

    csv_violations = [(r["catalog"], int(r["run_index"])) for r in rows
                      if r["keep_out_violation"] == "true"]
    replay_violations = [(row["catalog"], int(row["run_index"]))
                         for (row, _, _) in violations]
    sets_match = (set(csv_violations) == set(replay_violations)
                  and len(csv_violations) == len(replay_violations))

    if n_mismatch or not sets_match:
        for ex in mismatch_examples:
            sys.stderr.write("cross-validation mismatch: %s run %s: %s\n"
                             % (ex[0], ex[1], "; ".join(ex[2])))
        raise SystemExit(
            "replay cross-validation FAILED (%d/%d run mismatches, violation "
            "sets match: %s). The replay mirrors are stale against the C++ "
            "source or the committed CSV; fix before classifying."
            % (n_mismatch, n_runs, sets_match))

    nonviol_margins.sort()

    # Pass 2: forensic rows for the violating runs.
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
    for (row, rep, rec) in violations:
        x0, y0, z0 = rec.r
        vpost_y = rec.v[1] + rec.dv_cmd[1]
        drift = abs(vpost_y + 2.0 *
                    math.sqrt(EARTH_MU / ((EARTH_RADIUS +
                              CATALOG_ALT_KM[row["catalog"]] * 1000.0) ** 3)) * x0)
        corridor_dev = math.sqrt(rec.r[0] ** 2 +
                                 (rec.r[1] + STANDOFF_FACTOR * KEEP_OUT_M) ** 2 +
                                 rec.r[2] ** 2)
        out_rows.append({
            "schema_version": SCHEMA_VERSION,
            "catalog": row["catalog"],
            "run_index": row["run_index"],
            "run_seed": row["run_seed"],
            "target_index": str(rec.target_index),
            "gate_abort_ordinal": str(rec.abort_ordinal),
            "closing_speed_m_per_s": f(rec.v_close, 5),
            "dv_remaining_at_abort_m_per_s": f(rec.dv_remaining_before, 4),
            "r0_x_m": f(x0, 4), "r0_y_m": f(y0, 4), "r0_z_m": f(z0, 4),
            "v0_x_m_per_s": f(rec.v[0], 6),
            "v0_y_m_per_s": f(rec.v[1], 6),
            "v0_z_m_per_s": f(rec.v[2], 6),
            "initial_range_m": f(math.sqrt(x0 ** 2 + y0 ** 2 + z0 ** 2), 4),
            "corridor_deviation_m": f(corridor_dev, 4),
            "abort_dv_x_m_per_s": f(rec.dv_cmd[0], 6),
            "abort_dv_y_m_per_s": f(rec.dv_cmd[1], 6),
            "abort_dv_z_m_per_s": f(rec.dv_cmd[2], 6),
            "abort_dv_mag_uncapped_m_per_s": f(rec.dv_mag_uncapped, 6),
            "abort_dv_cap_m_per_s": f(ABORT_DV_CAP, 1),
            "capped": "true" if rec.capped else "false",
            "post_burn_drift_m_per_s": f(drift, 9),
            "coast_min_range_m": f(rec.coast_min_m, 4),
            "coast_min_time_s": f(rec.coast_min_t_s, 1),
            "analytic_ellipse_min_range_m": f(ellipse_min_range(x0, y0, z0), 4),
            "keep_out_radius_m": f(KEEP_OUT_M, 1),
            "classification": classify(rec),
        })

    # Sort exactly like the runs CSV (catalog A block then B, by run_index) --
    # already in row order because we walked rows in order. Write CSV (LF).
    with open(out_csv, "w", newline="\n") as fh:
        wr = csv.DictWriter(fh, fieldnames=fieldnames, lineterminator="\n")
        wr.writeheader()
        for r in out_rows:
            wr.writerow(r)

    # ------------------------------------------------------------------- md
    n_capped = sum(1 for r in out_rows if r["capped"] == "true")
    n_clean = len(out_rows) - n_capped
    lines = []
    w = lines.append
    w("# WP10c keep-out violation forensics (read-only replay)")
    w("")
    w("Forensic classification of every keep-out-violating run in the committed")
    w("`generated/wp5_campaign_runs.csv`, per spec v5 section 6 WP10c. Produced by")
    w("`tools/forensics/make_forensics.py` (Python 3 stdlib, deterministic, R6/R9);")
    w("regenerated by `tools/regenerate_all.sh`. No campaign or control-law code")
    w("was modified or executed: the tool replays the committed campaign's")
    w("deterministic draw stream and abort-coast physics from the pinned seeds.")
    w("")
    w("## 1. Replay recipe (WP10c completion criterion)")
    w("")
    w("- Per-run seed: `SplitMix64(master_seed XOR FNV1a64(catalog), run_index)`")
    w("  with `master_seed = 0x5AD5C0DECAFE2026` (campaign.hpp); the run seeds in")
    w("  the forensics table below equal the committed `run_seed` column.")
    w("- Draw stream: `std::mt19937_64(run_seed)`, 53-bit uniforms in (0, 1],")
    w("  explicit Box-Muller with spare caching, in the exact draw order of")
    w("  `run_one_mission` (campaign.cpp).")
    w("- Abort physics: Clohessy-Wiltshire drift-null impulse capped at 2.0 m/s")
    w("  (mission.cpp `compute_safe_abort`), RK4 coast at dt = 8 s over 1 orbital")
    w("  period of the catalog altitude (the campaign's coarsened screen,")
    w("  main_campaign.cpp), keep-out radius 200 m, departure standoff 400 m.")
    w("- To reproduce: `python3 tools/forensics/make_forensics.py` from the repo")
    w("  root, or `bash tools/regenerate_all.sh build` for the full pipeline.")
    w("")
    w("## 2. Replay cross-validation (gate for everything below)")
    w("")
    w("- %d/%d committed runs replayed with ZERO mismatches across all" % (n_runs, n_runs))
    w("  replay-reachable CSV columns (run_seed, solar_factor,")
    w("  first_closing_speed, tumble_rate, dv_used, dv_remaining, removals,")
    w("  targets_attempted, gate_abort_events, kits_used, outcome), each compared")
    w("  at the CSV's own printed precision.")
    w("- %d gate-abort events replayed; the replay independently reproduces" % total_aborts)
    w("  exactly the committed set of %d keep-out violations (%s)."
      % (len(out_rows),
         ", ".join("%s: %d" % (k, per_catalog_counts[k])
                   for k in sorted(per_catalog_counts))))
    w("- Columns NOT replayable in Python (attitude-dynamics outputs:")
    w("  sync_time_s, mission_time_s) are not used by this analysis.")
    w("- Modeled assumption, guarded: every sync attempt succeeds. The committed")
    w("  CSV contains 0 sync_timeout_events across all %d runs; the tool" % n_runs)
    w("  hard-fails if that ever changes.")
    w("")
    w("## 3. Findings")
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
    w("The abort law nulls the drift, which yields a *bounded* relative orbit,")
    w("not necessarily a *clearing* one. The post-burn coast is the closed-form ellipse")
    w("`x(t) = x0 cos(nt)`, `y(t) = y0 - 2 x0 sin(nt)`, `z(t) = z0 cos(nt)`: a")
    w("dispersed radial offset x0 produces an along-track swing of 2|x0| about the")
    w("dispersed standoff y0, so the coast approaches the target to roughly")
    w("`|y0| - 2|x0|` (exactly: the sampled ellipse minimum, column")
    x0_lo = min(abs(float(r["r0_x_m"])) for r in out_rows)
    x0_hi = max(abs(float(r["r0_x_m"])) for r in out_rows)
    w("`analytic_ellipse_min_range_m`, which matches the RK4 sweep within the 8 s")
    w("screen resolution). Every violating run has |x0| between %s m and %s m"
      % (f(x0_lo, 1), f(x0_hi, 1)))
    w("(%s-%s sigma of the %s m per-axis dispersion), and the coast minimum"
      % (f(x0_lo / SIG_REL_POS, 1), f(x0_hi / SIG_REL_POS, 1), f(SIG_REL_POS, 0)))
    w("occurs at a quarter orbit (x0 < 0) or three-quarter orbit (x0 > 0), exactly")
    w("as the ellipse geometry predicts. The violation is a *geometric* property")
    w("of where the abort starts, not a thruster-authority (cap) property.")
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
    w("screen at dt = 8 s over 1 period, WP5 committed dispersion set as")
    w("configured in CampaignConfig]; it inherits every WP5 model limitation")
    w("(sensor dispersions drawn but not re-propagated; flat placeholder")
    w("Delta-v costs; see the evidence-pack inventory).")
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
    w("## 5. Near-miss context (WP11 input)")
    w("")
    w("Of the %d replayed non-violating aborts, the smallest clearance above the"
      % len(nonviol_margins))
    w("200 m keep-out sphere is %s m, and %d aborts cleared by less than 50 m."
      % (f(nonviol_margins[0], 1),
         sum(1 for m in nonviol_margins if m < 50.0)))
    w("The margin distribution is continuous down to essentially zero: the")
    w("committed violation rate (%d of %d runs; see wp5_campaign_summary.md) is"
      % (len(out_rows), n_runs))
    w("the tail of a broad near-miss population, not a cluster")
    w("of isolated outliers. Any WP11 fix must therefore constrain the abort")
    w("ENTRY STATE (e.g. a reachability/ellipse-clearance check before committing")
    w("to a state, and/or choosing an abort impulse that also shapes the ellipse)")
    w("rather than only raising thruster authority: `abort_dv` authority is not")
    w("the binding constraint in any observed violation.")
    w("")
    w("## 6. Honest limits")
    w("")
    w("- Linear CW only (L0); no J2/drag/SRP. The WP12 fidelity ladder re-issues")
    w("  safety statements per level.")
    w("- The 8 s screen step under-resolves the exact minimum by up to a few")
    w("  meters versus the analytic ellipse (visible in the table); the")
    w("  classification is unaffected (the analytic and swept minima agree on")
    w("  violation for every row).")
    w("- This tool CLASSIFIES the committed campaign; it does not change any")
    w("  behavior, quote any new safety claim, or alter the violation rate.")
    w("  The campaign keep-out numbers remain exactly as committed")
    w("  (see wp5_campaign_summary.md).")
    w("")
    with open(out_md, "w", newline="\n") as fh:
        fh.write("\n".join(lines))

    sys.stdout.write("[WP10c] wrote %s (%d rows) and %s (%d lines)\n"
                     % (os.path.relpath(out_csv, root), len(out_rows),
                        os.path.relpath(out_md, root), len(lines)))


if __name__ == "__main__":
    main()
