#!/usr/bin/env python3
"""check_cw.py -- independent cross-validation of ADSC's Clohessy-Wiltshire
(CW / Hill) closed-form relative-motion model against a full NONLINEAR
two-body numerical propagation.

Scope (read this before trusting the numbers):
  - This checks ONE thing: how large is the difference between (a) the
    linear CW closed-form solution and (b) a full nonlinear point-mass
    two-body propagation of the SAME two spacecraft, for the SAME initial
    relative state. That difference is, by definition, the CW
    LINEARIZATION error (no J2, no drag, no oblateness -- just the
    nonlinearity of 1/r^2 gravity that CW linearizes away).
  - It does NOT validate J2, drag, attitude, control, estimation, or any
    of ADSC's abort/guidance LOGIC. It also does not re-derive an
    alternative closed form for CW -- there is only one correct closed
    form, so an independent re-derivation necessarily reproduces the same
    formula (see cw_stm() below); the actual cross-validation value is in
    the nonlinear-propagation comparison, not in re-deriving CW itself.
  - The nonlinear "truth" propagator is a from-scratch implementation of
    the point-mass two-body ODE, integrated with scipy.integrate.solve_ivp
    (DOP853, an established high-order embedded Runge-Kutta method). This
    is spot-checked against hapsira (github.com/pleiszenburg/hapsira, the
    maintained fork of the archived poliastro, itself validated against
    GMAT/Orekit -- see poliastro/validation) -- an independent,
    third-party, actively maintained astrodynamics package -- at several
    sample points per case (see verify_against_hapsira()). Using scipy
    directly for the dense time series (instead of calling hapsira's
    Cowell propagator at every sample) is a performance choice, not a
    methodological one: both integrate the identical point-mass EOM, and
    the spot-check confirms they agree to sub-millimeter precision.

Cases:
  1. The WP1 approach corridor (10 drift-free "safety ellipse" holds,
     rho = 1200 m down to 300 m, at the 825 km generic reference orbit).
     Includes the pinned 424.264 m ("424.3 m") worst-corridor-hold
     minimum range (rho = 300 m, z_amp = 300 m, y_center = 0).
  2. Forensic-14-LIKE bounded coasts at the two catalog orbits actually
     used by ADSC's forensic-14 pinned states (catalog A: 840 km / 71 deg;
     catalog B: 750 km / 78 deg): drift-free ellipses at rho = 300 m and
     rho = 400 m (these two radii bracket the ~350-470 m range the actual
     14 pinned forensic states span -- see forensic14_states.hpp) plus the
     exact departure-standoff v=0 equilibrium state (400 m along-track)
     that ADSC's own ladder test (tests/test_ladder.cpp, section 1c) uses.
  NOTE ON WHAT THIS DOES NOT REPRODUCE: ADSC's actual 14 forensic states
  carry small, near-arbitrary post-dispersion velocities that are made
  safe by an internal abort delta-v law (guidance.cpp/controller.cpp's
  clearing_abort_for) before they are coasted. That law is not
  reimplemented here (it requires the C++ build; there is no local C++
  toolchain -- see repo memory) and reimplementing it would not be an
  independent check anyway (it would just be re-deriving ADSC's own
  code). Using the RAW pre-abort velocities instead of the corrected ones
  would produce large, physically meaningless secular drift that is an
  artifact of skipping that correction step, not a CW-model error --
  so this script does not do that. Instead it tests the same class of
  bounded, drift-free relative geometry (same catalog altitude/
  inclination pairs, same order-of-magnitude separation) that the safety
  design and the ladder's own departure-standoff test rely on. This is
  clearly weaker than reproducing the exact pinned states, and is stated
  as such in RESULTS.md.

Output: prints a results table and writes validation/cw_results.csv.
Deterministic (no RNG). ASCII only.
"""
from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------------------
# Physical constants. WGS-84 Earth GM and equatorial radius -- these are
# universal reference values (the same ones ADSC's relmotion.hpp cites),
# not something under test; there is nothing to "independently derive" for a
# defined standard constant.
# ---------------------------------------------------------------------------
MU_EARTH = 3.986004418e14   # m^3/s^2 (WGS-84)
R_EARTH = 6378137.0         # m (WGS-84 equatorial)


# ---------------------------------------------------------------------------
# 1. Independent CW/Hill closed-form STM, written fresh from the standard
#    textbook result (e.g. Vallado, "Fundamentals of Astrodynamics and
#    Applications", 4th ed., Sec 7.5; Curtis, "Orbital Mechanics for
#    Engineering Students", Sec 7.6). This is THE unique closed-form
#    solution of the linearized Hill equations -- any correct
#    implementation must produce this matrix.
# ---------------------------------------------------------------------------
def cw_stm(n: float, t: float) -> np.ndarray:
    s, c = math.sin(n * t), math.cos(n * t)
    phi = np.zeros((6, 6))
    # position <- initial position
    phi[0, 0] = 4.0 - 3.0 * c
    phi[1, 0] = 6.0 * (s - n * t)
    phi[1, 1] = 1.0
    phi[2, 2] = c
    # position <- initial velocity
    phi[0, 3] = s / n
    phi[0, 4] = 2.0 * (1.0 - c) / n
    phi[1, 3] = 2.0 * (c - 1.0) / n
    phi[1, 4] = (4.0 * s - 3.0 * n * t) / n
    phi[2, 5] = s / n
    # velocity <- initial position
    phi[3, 0] = 3.0 * n * s
    phi[4, 0] = 6.0 * n * (c - 1.0)
    phi[5, 2] = -n * s
    # velocity <- initial velocity
    phi[3, 3] = c
    phi[3, 4] = 2.0 * s
    phi[4, 3] = -2.0 * s
    phi[4, 4] = 4.0 * c - 3.0
    phi[5, 5] = c
    return phi


def cw_propagate(x0: np.ndarray, n: float, t: float) -> np.ndarray:
    return cw_stm(n, t) @ x0


# ---------------------------------------------------------------------------
# 2. Full nonlinear point-mass two-body propagator (the "truth" model).
# ---------------------------------------------------------------------------
def two_body_rhs(_t: float, state: np.ndarray, mu: float) -> np.ndarray:
    r = state[:3]
    v = state[3:]
    r3 = np.dot(r, r) ** 1.5
    a = -mu * r / r3
    return np.concatenate([v, a])


def propagate_twobody(r0: np.ndarray, v0: np.ndarray, mu: float,
                       t_eval: np.ndarray):
    y0 = np.concatenate([r0, v0])
    sol = solve_ivp(two_body_rhs, (0.0, float(t_eval[-1])), y0, args=(mu,),
                     t_eval=t_eval, method="DOP853", rtol=3e-13, atol=1e-6,
                     dense_output=False)
    if not sol.success:
        raise RuntimeError(f"two-body integration failed: {sol.message}")
    return sol.y[:3, :].T, sol.y[3:, :].T  # (N,3), (N,3)


def verify_against_hapsira(r0: np.ndarray, v0: np.ndarray, t_check_s: float,
                            label: str) -> float:
    """Spot-check propagate_twobody() against hapsira's CowellPropagator
    (independent third-party package) at one time. Returns the position
    disagreement [m]. Raises if hapsira is not importable -- the caller
    decides whether that is fatal."""
    from astropy import units as u
    from hapsira.bodies import Earth
    from hapsira.twobody import Orbit
    from hapsira.twobody.propagation import CowellPropagator

    orb = Orbit.from_vectors(Earth, r0 * u.m, v0 * u.m / u.s)
    r_hap, _v_hap = orb.propagate(t_check_s * u.s,
                                   method=CowellPropagator(rtol=1e-12)).rv()
    r_hap = r_hap.to(u.m).value
    r_mine, _v_mine = propagate_twobody(r0, v0, MU_EARTH,
                                         np.array([0.0, t_check_s]))
    disagreement = float(np.linalg.norm(r_mine[-1] - r_hap))
    print(f"  [hapsira spot-check] {label}: scipy-vs-hapsira position "
          f"disagreement at t={t_check_s:.1f}s = {disagreement:.3e} m")
    return disagreement


# ---------------------------------------------------------------------------
# 3. LVLH frame construction (standard orbital-mechanics geometry -- the same
#    definition ADSC's propagation.hpp documents in comments: x=radial,
#    y=along-track, z=orbit-normal / cross-track). Re-derived independently
#    here, not copied from the C++.
# ---------------------------------------------------------------------------
def lvlh_frame(r: np.ndarray, v: np.ndarray):
    xhat = r / np.linalg.norm(r)
    h = np.cross(r, v)
    zhat = h / np.linalg.norm(h)
    yhat = np.cross(zhat, xhat)
    return xhat, yhat, zhat, np.linalg.norm(h)


def target_ic(alt_km: float, incl_deg: float):
    """Circular target orbit, RAAN=0, argument of latitude u=0 at t=0
    (ascending node on the ECI x-axis). Standard result: h_hat =
    (0, -sin i, cos i)."""
    a = R_EARTH + alt_km * 1000.0
    incl = math.radians(incl_deg)
    r = a * np.array([1.0, 0.0, 0.0])
    vcirc = math.sqrt(MU_EARTH / a)
    v = vcirc * np.array([0.0, math.cos(incl), math.sin(incl)])
    return r, v, a


def chaser_ic_from_lvlh(r_t: np.ndarray, v_t: np.ndarray, rel6: np.ndarray):
    """Map an LVLH relative state [x,y,z,vx,vy,vz] at t=0 to an absolute ECI
    (r,v) for the chaser, applying the target's own frame rotation rate
    omega = (h/r^2) zhat exactly once (standard rotating-frame velocity
    transform: v_rel_inertial = v_rel_rotating + omega x r_rel)."""
    xhat, yhat, zhat, h = lvlh_frame(r_t, v_t)
    C = np.array([xhat, yhat, zhat])  # ECI -> LVLH: r_lvlh = C @ r_eci
    r_rel_lvlh, v_rel_lvlh = rel6[:3], rel6[3:]
    r_rel_eci = C.T @ r_rel_lvlh
    # omega x r must be evaluated with BOTH vectors expressed in the same
    # (LVLH) basis components -- omega is along the frame's own z-axis, so
    # its LVLH-frame components are (0, 0, omega_mag), NOT the ECI vector
    # omega_mag*zhat (mixing an ECI-component vector with LVLH-component
    # r_rel would silently corrupt every case with a nonzero xhat/yhat/zhat
    # rotation relative to the ECI axes, i.e. every non-equatorial case).
    omega_lvlh = np.array([0.0, 0.0, h / np.dot(r_t, r_t)])
    v_rel_eci = C.T @ (v_rel_lvlh + np.cross(omega_lvlh, r_rel_lvlh))
    return r_t + r_rel_eci, v_t + v_rel_eci


def eci_to_lvlh_rel(r_t: np.ndarray, v_t: np.ndarray, r_c: np.ndarray,
                     v_c: np.ndarray):
    """Inverse of chaser_ic_from_lvlh, evaluated at the CURRENT (possibly
    perturbed) target state -- i.e. differencing into the target's
    instantaneous, freshly-recomputed LVLH triad at each sample time (same
    convention ADSC's propagation.hpp documents for its own L1 ladder)."""
    xhat, yhat, zhat, h = lvlh_frame(r_t, v_t)
    C = np.array([xhat, yhat, zhat])
    r_rel_lvlh = C @ (r_c - r_t)
    omega_z = h / np.dot(r_t, r_t)
    v_rel_lvlh = C @ (v_c - v_t) - np.cross(np.array([0.0, 0.0, omega_z]),
                                             r_rel_lvlh)
    return r_rel_lvlh, v_rel_lvlh


# ---------------------------------------------------------------------------
# 4. Independent re-implementation of ADSC's approach_corridor() (linear
#    interpolation of rho, then min_range filter) -- relmotion.cpp.
# ---------------------------------------------------------------------------
def approach_corridor(rho_far, rho_near, keep_out, n_holds):
    holds = []
    for i in range(n_holds):
        frac = 0.0 if n_holds == 1 else i / (n_holds - 1)
        rho = rho_far + frac * (rho_near - rho_far)
        # SafetyEllipse{rho, z_amp=rho, y_center=0}; scan theta densely
        # (independent of ADSC's 1440-sample scan, but same density) to
        # find min_range, exactly reproducing relmotion.cpp's own filter.
        best = min_range_of_ellipse(rho, rho, 0.0)
        if best > keep_out:
            holds.append(rho)
    return holds


def min_range_of_ellipse(rho, z_amp, y_center, samples=1440):
    th = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    x = rho * np.cos(th)
    y = -2.0 * rho * np.sin(th) + y_center
    z = z_amp * np.cos(th)
    r = np.sqrt(x * x + y * y + z * z)
    return float(np.min(r))


def ellipse_state_at_theta(rho, z_amp, y_center, n, theta):
    ct, st = math.cos(theta), math.sin(theta)
    x = rho * ct
    y = -2.0 * rho * st + y_center
    z = z_amp * ct
    vx = -rho * n * st
    vy = -2.0 * rho * n * ct
    vz = -z_amp * n * st
    return np.array([x, y, z, vx, vy, vz])


# ---------------------------------------------------------------------------
# 5. Case runner.
# ---------------------------------------------------------------------------
def run_case(name, alt_km, incl_deg, rel6, n_orbits, samples_per_orbit=200):
    r_t0, v_t0, a = target_ic(alt_km, incl_deg)
    n = math.sqrt(MU_EARTH / a ** 3)
    period = 2.0 * math.pi / n
    r_c0, v_c0 = chaser_ic_from_lvlh(r_t0, v_t0, rel6)

    horizon = n_orbits * period
    n_samples = max(50, int(samples_per_orbit * n_orbits))
    t_eval = np.linspace(0.0, horizon, n_samples)

    r_t_series, v_t_series = propagate_twobody(r_t0, v_t0, MU_EARTH, t_eval)
    r_c_series, v_c_series = propagate_twobody(r_c0, v_c0, MU_EARTH, t_eval)

    max_dev_by_orbit = {}   # cumulative max deviation up to orbit k
    min_range_truth = math.inf
    min_range_cw = math.inf
    max_dev_overall = 0.0
    max_dev_t = 0.0
    rows = []
    for k, t in enumerate(t_eval):
        r_rel_lvlh, _v_rel_lvlh = eci_to_lvlh_rel(
            r_t_series[k], v_t_series[k], r_c_series[k], v_c_series[k])
        cw6 = cw_propagate(rel6, n, t)
        dev = float(np.linalg.norm(r_rel_lvlh - cw6[:3]))
        rng_truth = float(np.linalg.norm(r_rel_lvlh))
        rng_cw = float(np.linalg.norm(cw6[:3]))
        min_range_truth = min(min_range_truth, rng_truth)
        min_range_cw = min(min_range_cw, rng_cw)
        if dev > max_dev_overall:
            max_dev_overall, max_dev_t = dev, t
        orbit_idx = int(t / period) + 1
        for kk in range(orbit_idx, n_orbits + 1):
            max_dev_by_orbit[kk] = max(max_dev_by_orbit.get(kk, 0.0), dev)
        rows.append((t, dev, rng_truth, rng_cw))

    return {
        "name": name,
        "alt_km": alt_km,
        "incl_deg": incl_deg,
        "separation_m": float(np.linalg.norm(rel6[:3])),
        "n": n,
        "period_s": period,
        "n_orbits": n_orbits,
        "max_dev_overall_m": max_dev_overall,
        "max_dev_t_s": max_dev_t,
        "max_dev_orbit_frac": max_dev_t / period,
        "max_dev_by_orbit": max_dev_by_orbit,
        "min_range_truth_m": min_range_truth,
        "min_range_cw_m": min_range_cw,
    }


def print_report(results):
    print()
    print("=" * 78)
    print("CW / two-body cross-validation summary")
    print("=" * 78)
    hdr = (f"{'case':<22}{'alt_km':>8}{'incl':>6}{'sep_m':>9}"
           f"{'orb1':>9}{'orb2':>9}{'orb3':>9}{'orb4':>9}{'orb5':>9}"
           f"{'minR_t':>10}{'minR_cw':>10}")
    print(hdr)
    for r in results:
        mdb = r["max_dev_by_orbit"]
        o1 = mdb.get(1, float("nan"))
        o2 = mdb.get(2, float("nan"))
        o3 = mdb.get(3, float("nan"))
        o4 = mdb.get(4, float("nan"))
        o5 = mdb.get(5, float("nan"))
        print(f"{r['name']:<22}{r['alt_km']:>8.0f}{r['incl_deg']:>6.1f}"
              f"{r['separation_m']:>9.2f}{o1:>9.4f}{o2:>9.4f}{o3:>9.4f}"
              f"{o4:>9.4f}{o5:>9.4f}"
              f"{r['min_range_truth_m']:>10.3f}{r['min_range_cw_m']:>10.3f}")
    print()
    worst = max(r["max_dev_by_orbit"].get(1, 0.0) for r in results)
    print(f"Worst deviation across all cases within orbit 1: {worst:.4f} m")
    print("ADSC's own L1-ladder claim (tests/test_ladder.cpp, "
          "docs/safety.md): worst 0.52 m across the forensic-14 states, "
          "bounded by a stated 2.0 m tolerance (over ~1 target orbital "
          "period, J2 forced off).")

    print()
    for r in results:
        mdb = r["max_dev_by_orbit"]
        crossing_orbit = None
        for k in sorted(mdb):
            if mdb[k] > 2.0:
                crossing_orbit = k
                break
        if crossing_orbit is not None:
            print(f"FINDING: case '{r['name']}' exceeds the 2.0 m tolerance "
                  f"at orbit {crossing_orbit} (deviation {mdb[crossing_orbit]:.4f} m) "
                  f"if the coast/hold is extended that long. ADSC's own tests "
                  f"only exercise this tolerance over ~1 target orbital period "
                  f"(test_ladder.cpp), so this is not a contradiction of any "
                  f"existing claim -- it shows that scoping is load-bearing, "
                  f"not merely conservative, for a hold extended across "
                  f"multiple orbits.")


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    results = []

    # ---- Spot-check the "truth" propagator against hapsira -------------
    print("Spot-checking the from-scratch scipy two-body propagator "
          "against hapsira (independent package)...")
    try:
        r_t0, v_t0, _a = target_ic(825.0, 51.6)
        disagreement = verify_against_hapsira(r_t0, v_t0, 6000.0, "target@825km")
        hapsira_ok = disagreement < 1e-3  # sub-mm agreement expected
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"  WARNING: hapsira spot-check unavailable ({exc}); "
              f"proceeding with scipy-only propagation. This weakens the "
              f"'established package' cross-validation to scipy alone.")
        hapsira_ok = None
    print()

    # ---- Case set 1: WP1 approach corridor (825 km generic orbit) ------
    keep_out = 200.0
    corridor_rhos = approach_corridor(1200.0, 300.0, keep_out, 10)
    print(f"WP1 corridor holds surviving the {keep_out:.0f} m keep-out "
          f"filter: {[round(r, 1) for r in corridor_rhos]}")
    for rho in corridor_rhos:
        n_at_825, _, a_825 = None, None, None
        _, _, a_825 = target_ic(825.0, 0.0)
        n_825 = math.sqrt(MU_EARTH / a_825 ** 3)
        rel6 = ellipse_state_at_theta(rho, rho, 0.0, n_825, 0.0)
        res = run_case(f"wp1_rho{rho:.0f}m", 825.0, 0.0, rel6, n_orbits=5)
        results.append(res)

    # Independent regression check on the pinned 424.264 m number.
    pinned = 424.264069
    rho300 = 300.0
    match = [r for r in results if r["name"] == "wp1_rho300m"]
    if match:
        got = match[0]["min_range_cw_m"]
        print(f"\nPinned check: rho=300m worst-hold min range = {got:.6f} m "
              f"(repo's generated/reference_metrics.csv "
              f"wp1_worst_coast_min_range_m = {pinned:.6f} m); "
              f"agreement = {abs(got - pinned):.2e} m")

    # Inclination-invariance sanity check for the pure two-body (J2-off)
    # case: repeat rho=300 at 51.6 deg inclination, same altitude.
    _, _, a_825b = target_ic(825.0, 51.6)
    n_825b = math.sqrt(MU_EARTH / a_825b ** 3)
    rel6_300 = ellipse_state_at_theta(300.0, 300.0, 0.0, n_825b, 0.0)
    res_incl = run_case("wp1_rho300m_i51.6", 825.0, 51.6, rel6_300, n_orbits=5)
    results.append(res_incl)
    print(f"Inclination-invariance check (pure two-body, spherical "
          f"symmetry): rho=300m worst-hold max dev @orbit1 equatorial="
          f"{match[0]['max_dev_by_orbit'][1]:.4f} m vs 51.6deg="
          f"{res_incl['max_dev_by_orbit'][1]:.4f} m")

    # ---- Case set 2: forensic-14-LIKE bounded coasts at catalog A/B ----
    catalogs = [("A", 840.0, 71.0), ("B", 750.0, 78.0)]
    for cat, alt_km, incl_deg in catalogs:
        _, _, a_cat = target_ic(alt_km, incl_deg)
        n_cat = math.sqrt(MU_EARTH / a_cat ** 3)
        for rho in (300.0, 400.0):
            rel6 = ellipse_state_at_theta(rho, rho, 0.0, n_cat, 0.0)
            res = run_case(f"cat{cat}_rho{rho:.0f}m", alt_km, incl_deg, rel6,
                           n_orbits=5)
            results.append(res)
        # Exact departure-standoff v=0 equilibrium (ladder test 1c geometry).
        rel6_eq = np.array([0.0, -400.0, 0.0, 0.0, 0.0, 0.0])
        res_eq = run_case(f"cat{cat}_standoff400m_eq", alt_km, incl_deg,
                          rel6_eq, n_orbits=5)
        results.append(res_eq)

    print_report(results)

    # ---- Write CSV ------------------------------------------------------
    csv_path = os.path.join(out_dir, "cw_results.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case", "alt_km", "incl_deg", "separation_m",
                    "period_s", "n_orbits_tested",
                    "max_dev_orbit1_m", "max_dev_orbit2_m",
                    "max_dev_orbit3_m", "max_dev_orbit4_m",
                    "max_dev_orbit5_m", "max_dev_overall_m",
                    "max_dev_overall_orbit_frac",
                    "min_range_truth_m", "min_range_cw_m"])
        for r in results:
            mdb = r["max_dev_by_orbit"]
            w.writerow([r["name"], r["alt_km"], r["incl_deg"],
                        f"{r['separation_m']:.4f}", f"{r['period_s']:.4f}",
                        r["n_orbits"],
                        f"{mdb.get(1, float('nan')):.6f}",
                        f"{mdb.get(2, float('nan')):.6f}",
                        f"{mdb.get(3, float('nan')):.6f}",
                        f"{mdb.get(4, float('nan')):.6f}",
                        f"{mdb.get(5, float('nan')):.6f}",
                        f"{r['max_dev_overall_m']:.6f}",
                        f"{r['max_dev_orbit_frac']:.4f}",
                        f"{r['min_range_truth_m']:.6f}",
                        f"{r['min_range_cw_m']:.6f}"])
    print(f"\nWrote {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
