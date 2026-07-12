#!/usr/bin/env python3
"""check_decay.py -- independent cross-validation of ADSC's Vallado
exponential-atmosphere quasi-circular decay model (src/decay.cpp) against
an INDEPENDENT empirical atmosphere model (NRLMSISE-00 / MSIS2, via the
`pymsis` package -- github.com/SWxTREC/pymsis, the actively-maintained
Python interface to the NRL/CU-LASP NRLMSISE-00 and MSIS2 Fortran models).

This script does TWO separate things and is careful not to conflate them
(see RESULTS.md "what this does / does not validate"):

  PART 1 -- INTEGRATOR / ARITHMETIC CHECK (same atmosphere model).
  Re-implements ADSC's own Vallado Table 8-4 piecewise-exponential density
  bands from scratch in Python (the numbers are cited textbook constants,
  ported rather than re-derived -- there is nothing to "independently
  derive" for a published density-vs-altitude table) and re-integrates the
  SAME decay ODE with scipy.integrate.quad (adaptive Gauss-Kronrod), an
  entirely different numerical method from ADSC's C++ fixed-step
  trapezoidal rule (decay.hpp, 6000 steps). Agreement here validates that
  ADSC's C++ implementation computes the formula/integral correctly; it
  says NOTHING about whether the Vallado exponential atmosphere itself is
  a good model of the real thermosphere.

  PART 2 -- ATMOSPHERE MODEL CHECK (independent density source).
  Replaces the Vallado bands with an orbit-and-local-time-averaged
  NRLMSISE-00 density profile (same decay ODE, same integration bounds and
  stop altitude) and reports how much the decay-time estimate shifts. This
  is the actual "independent atmosphere implementation" cross-validation
  the task calls for. Any large shift here is a real, reportable
  atmosphere-MODEL disagreement -- not an integrator bug -- and is
  reported honestly, not tuned away. This part also computes a dedicated
  "shared moderate reference point" decay-years table (Vallado at
  solar_factor=1.0 "nominal" vs NRLMSISE-00 at F10.7=F10.7a=150, Ap=4) for
  both catalogs and both sail areas, written to
  decay_part2_reference_point.csv -- this is the table RESULTS.md's
  honest-interpretation FINDING is anchored on, so it must be computed
  here rather than only asserted in prose.

STELA (CNES's operational Java/GUI lifetime tool) was NOT run. It requires
a desktop Java GUI session and is out of scope for a scripted, one-time,
pinned-dependency Python study; see RESULTS.md for what STELA would add
beyond this check (full numerical orbit propagation with a real
NRLMSISE-00/JB2008 call and a real F10.7/Ap time history, rather than the
quasi-circular closed-form decay ODE with a static solar-activity proxy
both ADSC and this script use).

ASCII only, deterministic (fixed solar-activity proxies passed explicitly
to pymsis so no historical-index network lookup is triggered; fixed
epoch date, since date does not matter once F10.7/F10.7a/Ap are pinned
except through the diurnal/longitude sampling grid below, which is also
fixed).
"""
from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np
from scipy.integrate import quad

MU_EARTH = 3.986004418e14
R_EARTH = 6378137.0
SEC_PER_YEAR = 365.25 * 86400.0

# ADSC Config defaults (include/adsc/mission.hpp) needed to reproduce the
# committed generated/wp3_decay_trade.csv rows exactly. Ported (not
# re-derived -- these are simulation input parameters, not physics).
KIT_MASS_KG = 2.4
DRAG_CD = 2.2
STOP_ALTITUDE_KM = 180.0
SOLAR_MIN_FACTOR = 0.5
SOLAR_MAX_FACTOR = 8.0

CATALOGS = {
    "A": {"name": "SL-16 / Zenit-2 second stage", "mass_kg": 9000.0,
          "altitude_km": 840.0, "incl_deg": 71.0},
    "B": {"name": "SL-8 / Kosmos-3M second stage", "mass_kg": 1400.0,
          "altitude_km": 750.0, "incl_deg": 78.0},
}
SAIL_AREAS_M2 = (100.0, 1000.0)

# ---------------------------------------------------------------------------
# PART 1: Vallado Table 8-4 exponential atmosphere (ported constants, cited
# Vallado "Fundamentals of Astrodynamics and Applications" 4th ed.) -- the
# SAME table src/decay.cpp uses, typed in independently here.
# ---------------------------------------------------------------------------
VALLADO_BANDS = [
    (0.0, 1.225, 7.249), (25.0, 3.899e-2, 6.349), (30.0, 1.774e-2, 6.682),
    (40.0, 3.972e-3, 7.554), (50.0, 1.057e-3, 8.382), (60.0, 3.206e-4, 7.714),
    (70.0, 8.770e-5, 6.549), (80.0, 1.905e-5, 5.799), (90.0, 3.396e-6, 5.382),
    (100.0, 5.297e-7, 5.877), (110.0, 9.661e-8, 7.263), (120.0, 2.438e-8, 9.473),
    (130.0, 8.484e-9, 12.636), (140.0, 3.845e-9, 16.149), (150.0, 2.070e-9, 22.523),
    (180.0, 5.464e-10, 29.740), (200.0, 2.789e-10, 37.105), (250.0, 7.248e-11, 45.546),
    (300.0, 2.418e-11, 53.628), (350.0, 9.518e-12, 53.298), (400.0, 3.725e-12, 58.515),
    (450.0, 1.585e-12, 60.828), (500.0, 6.967e-13, 63.822), (600.0, 1.454e-13, 71.835),
    (700.0, 3.614e-14, 88.667), (800.0, 1.170e-14, 124.64), (900.0, 5.245e-15, 181.05),
    (1000.0, 3.019e-15, 268.00),
]


def vallado_density(alt_m: float, solar_factor: float) -> float:
    h_km = max(alt_m / 1000.0, 0.0)
    band = VALLADO_BANDS[0]
    for b in VALLADO_BANDS:
        if h_km >= b[0]:
            band = b
        else:
            break
    h0, rho0, scale_h = band
    return rho0 * math.exp(-(h_km - h0) / scale_h) * solar_factor


def decay_years_vallado(mass_kg, alt_km, sail_area_m2, solar_factor,
                         stop_km=STOP_ALTITUDE_KM, kit_mass_kg=KIT_MASS_KG,
                         cd=DRAG_CD) -> float:
    m_total = mass_kg + kit_mass_kg
    aom = sail_area_m2 / m_total
    a0 = R_EARTH + alt_km * 1000.0
    a_stop = R_EARTH + stop_km * 1000.0
    # Help the adaptive quadrature at the band-edge kinks (density is
    # continuous but not smooth there).
    band_points = [R_EARTH + b[0] * 1000.0 for b in VALLADO_BANDS
                   if a_stop < R_EARTH + b[0] * 1000.0 < a0]

    def integrand(a):
        rho = vallado_density(a - R_EARTH, solar_factor)
        return 1.0 / (rho * cd * aom * math.sqrt(MU_EARTH * a))

    val, _err = quad(integrand, a_stop, a0, points=band_points, limit=400)
    return val / SEC_PER_YEAR


# ---------------------------------------------------------------------------
# PART 2: NRLMSISE-00 (pymsis) orbit-and-local-time-averaged density.
# ---------------------------------------------------------------------------
def pymsis_density_profile(alt_grid_km: np.ndarray, incl_deg: float,
                            f107: float, f107a: float, ap: float,
                            msis_version=0, n_lon=8, n_phase=8) -> np.ndarray:
    """Orbit-averaged density-vs-altitude, approximating the density a
    satellite on a circular orbit of the given inclination samples over
    many orbits: average over n_lon longitudes (proxy for sampling every
    local solar time as the orbit precesses / Earth rotates underneath)
    and n_phase orbital phases (mapped to the achievable latitude range
    via sin(lat) = sin(i) sin(u), same relation used in
    wp13-edt-derivation.md Section 2.2 for the magnetic-latitude analog).
    This is a documented simplification -- see RESULTS.md -- not a true
    time-weighted orbit average along a real trajectory."""
    from pymsis import msis, Variable

    lons = np.linspace(0.0, 360.0, n_lon, endpoint=False)
    us = np.linspace(0.0, 2.0 * math.pi, n_phase, endpoint=False)
    lats = np.degrees(np.arcsin(np.sin(math.radians(incl_deg)) * np.sin(us)))
    out = msis.calculate(["2026-07-01"], lons, lats, alt_grid_km,
                         f107s=[f107], f107as=[f107a], aps=[[ap] * 7],
                         version=msis_version)
    dens = out[0, :, :, :, Variable.MASS_DENSITY]  # (n_lon, n_phase, n_alt)
    return dens.mean(axis=(0, 1))


def decay_years_pymsis(mass_kg, alt_km, incl_deg, sail_area_m2, f107, f107a,
                       ap, stop_km=STOP_ALTITUDE_KM, kit_mass_kg=KIT_MASS_KG,
                       cd=DRAG_CD, msis_version=0, n_grid=4000) -> float:
    m_total = mass_kg + kit_mass_kg
    aom = sail_area_m2 / m_total
    a0 = R_EARTH + alt_km * 1000.0
    a_stop = R_EARTH + stop_km * 1000.0
    a_grid = np.linspace(a_stop, a0, n_grid)
    alt_grid_km = (a_grid - R_EARTH) / 1000.0
    rho = pymsis_density_profile(alt_grid_km, incl_deg, f107, f107a, ap,
                                 msis_version=msis_version)
    integrand = 1.0 / (rho * cd * aom * np.sqrt(MU_EARTH * a_grid))
    val = np.trapz(integrand, a_grid)
    return val / SEC_PER_YEAR


# Solar-activity proxies. There is no single universally standardized
# "solar min/max" (F10.7, Ap) pair; these are commonly-cited bracketing
# values (quiet-sun minimum ~65-70 sfu / Ap~4; a representative elevated
# but not extreme-storm maximum ~200 sfu / Ap~15). A sensitivity case at
# F10.7=250 (near the highest historically observed monthly-mean values)
# is also reported. Reported honestly as a documented choice, not derived.
SOLAR_PROXIES = {
    "quiet (F10.7=70, Ap=4)": (70.0, 70.0, 4.0),
    "active (F10.7=200, Ap=15)": (200.0, 200.0, 15.0),
    "very_active (F10.7=250, Ap=15)": (250.0, 250.0, 15.0),
}


def load_committed_csv(path):
    rows = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row["record_type"] != "decay_years":
                continue
            key = (row["catalog"], float(row["sail_area_m2"]))
            rows[key] = (float(row["value_solar_max"]),
                        float(row["value_solar_min"]))
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(here, "..", "generated", "wp3_decay_trade.csv")
    committed = load_committed_csv(csv_path)

    print("=" * 78)
    print("PART 1: integrator/arithmetic check (SAME Vallado atmosphere,")
    print("independent scipy.integrate.quad vs ADSC's C++ trapezoidal rule)")
    print("=" * 78)
    part1_rows = []
    for cat_id, cat in CATALOGS.items():
        for area in SAIL_AREAS_M2:
            y_max = decay_years_vallado(cat["mass_kg"], cat["altitude_km"],
                                        area, SOLAR_MAX_FACTOR)
            y_min = decay_years_vallado(cat["mass_kg"], cat["altitude_km"],
                                        area, SOLAR_MIN_FACTOR)
            key = (cat["name"], area)
            committed_max, committed_min = committed.get(key, (float("nan"),) * 2)
            rel_err_max = abs(y_max - committed_max) / committed_max
            rel_err_min = abs(y_min - committed_min) / committed_min
            print(f"catalog {cat_id} area={area:>6.0f} m2: "
                  f"quad max={y_max:10.4f}yr (committed {committed_max:10.4f}, "
                  f"rel.err {rel_err_max:.2e})  "
                  f"quad min={y_min:10.4f}yr (committed {committed_min:10.4f}, "
                  f"rel.err {rel_err_min:.2e})")
            part1_rows.append([cat_id, area, y_max, committed_max, rel_err_max,
                               y_min, committed_min, rel_err_min])
    worst_rel_err = max(max(r[4], r[7]) for r in part1_rows)
    print(f"\nWorst relative disagreement (integrator/arithmetic check): "
          f"{worst_rel_err:.2e}")

    print()
    print("=" * 78)
    print("PART 2: atmosphere-MODEL check (Vallado exponential table vs")
    print("independent NRLMSISE-00 [pymsis], SAME decay ODE + integration")
    print("bounds, orbit-and-local-time-averaged density)")
    print("=" * 78)
    part2_rows = []
    try:
        import pymsis  # noqa: F401
        pymsis_ok = True
    except Exception as exc:  # pragma: no cover
        print(f"WARNING: pymsis unavailable ({exc}); skipping Part 2.")
        pymsis_ok = False

    if pymsis_ok:
        # Reference single-point sanity check: Vallado at solar_factor=1.0
        # ("nominal", no min/max scaling) vs pymsis at a common moderate
        # proxy (F10.7=150), same altitude, to characterize the baseline
        # atmosphere-model gap independent of ADSC's own min/max bracket
        # choice.
        for alt_km in (400.0, 750.0, 840.0):
            v_nom = vallado_density(alt_km * 1000.0, 1.0)
            p_nom = pymsis_density_profile(np.array([alt_km]), 51.6, 150.0,
                                           150.0, 4.0)[0]
            print(f"  spot check @ {alt_km:.0f} km: Vallado(nominal, "
                  f"solar_factor=1.0) = {v_nom:.4e} kg/m3 vs "
                  f"NRLMSISE-00(F10.7=150,Ap=4) = {p_nom:.4e} kg/m3  "
                  f"(ratio Vallado/MSIS = {v_nom / p_nom:.2f}x)")
        print()

        # Reference-point DECAY-YEARS check (the actual numbers cited in
        # RESULTS.md's "shared moderate reference point" table): Vallado at
        # solar_factor=1.0 ("nominal", no min/max bracket applied) vs
        # NRLMSISE-00 at the same common moderate proxy (F10.7=F10.7a=150,
        # Ap=4) used in the density spot check above, for both catalogs and
        # both sail areas. This is the decay-TIME analog of the density
        # spot check, and is what the honest-interpretation FINDING below
        # actually cites -- it must be computed here, not just asserted.
        print("Reference-point decay-years check (Vallado nominal "
              "solar_factor=1.0 vs NRLMSISE-00 @ F10.7=F10.7a=150, Ap=4 -- "
              "the shared moderate point the FINDING below is anchored on):")
        refpoint_rows = []
        for cat_id, cat in CATALOGS.items():
            for area in SAIL_AREAS_M2:
                y_nom = decay_years_vallado(cat["mass_kg"], cat["altitude_km"],
                                            area, 1.0)
                y_150 = decay_years_pymsis(cat["mass_kg"], cat["altitude_km"],
                                           cat["incl_deg"], area, 150.0, 150.0,
                                           4.0)
                ratio = y_150 / y_nom
                print(f"catalog {cat_id} area={area:>6.0f} m2: Vallado "
                      f"nominal = {y_nom:10.4f} yr, NRLMSISE-00(F10.7=150) = "
                      f"{y_150:10.4f} yr, ratio(MSIS/Vallado) = {ratio:.3f}")
                refpoint_rows.append([cat_id, area, y_nom, y_150, ratio])
        print()

        for cat_id, cat in CATALOGS.items():
            for area in SAIL_AREAS_M2:
                key = (cat["name"], area)
                v_max, v_min = committed.get(key, (float("nan"),) * 2)
                for label, (f107, f107a, ap) in SOLAR_PROXIES.items():
                    y_p = decay_years_pymsis(cat["mass_kg"], cat["altitude_km"],
                                             cat["incl_deg"], area, f107, f107a, ap)
                    print(f"catalog {cat_id} area={area:>6.0f} m2 [{label:<28}]: "
                          f"NRLMSISE-00 decay = {y_p:10.4f} yr")
                    part2_rows.append([cat_id, area, label, f107, f107a, ap, y_p,
                                       v_max, v_min])
                print(f"  (ADSC Vallado band for comparison: "
                      f"{v_min:.4f} yr [solar min] .. {v_max:.4f} yr [solar max])")

    print()
    print("STELA (CNES): NOT RUN. Desktop Java GUI tool, out of scope for a "
          "scripted pinned-dependency study -- see RESULTS.md.")

    # ---- write CSVs -------------------------------------------------------
    with open(os.path.join(here, "decay_part1_integrator_check.csv"), "w",
              newline="") as f:
        w = csv.writer(f)
        w.writerow(["catalog", "sail_area_m2", "quad_years_max",
                    "committed_years_max", "rel_err_max", "quad_years_min",
                    "committed_years_min", "rel_err_min"])
        w.writerows(part1_rows)

    if pymsis_ok:
        with open(os.path.join(here, "decay_part2_atmosphere_check.csv"), "w",
                  newline="") as f:
            w = csv.writer(f)
            w.writerow(["catalog", "sail_area_m2", "solar_proxy", "f107",
                        "f107a", "ap", "nrlmsise00_years",
                        "adsc_vallado_years_solar_max",
                        "adsc_vallado_years_solar_min"])
            w.writerows(part2_rows)

        with open(os.path.join(here, "decay_part2_reference_point.csv"), "w",
                  newline="") as f:
            w = csv.writer(f)
            w.writerow(["catalog", "sail_area_m2", "vallado_nominal_years",
                        "nrlmsise00_f107_150_years", "ratio_msis_over_vallado"])
            w.writerows(refpoint_rows)
    print(f"\nWrote CSVs under {here}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
