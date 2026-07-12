#!/usr/bin/env python3
"""check_edt_eta.py -- independent numerical verification of the EDT
orbit-averaged inclination-efficiency formulas used by ADSC's WP13
electrodynamic-tether model (src/decay.cpp: eta_hi = |cos i|,
eta_lo = cos(i)^2; derived in wp13-edt-derivation.md).

Method (independent of wp13-edt-derivation.md's own Section 8.2 Gauss-
Legendre pseudocode -- this uses plain numpy trapezoidal quadrature over a
dense uniform grid instead, and derives B_n from the full 3D dipole field
vector rather than assuming the analytic B_n = beta*cos(i) simplification
up front):

  1. Build the aligned-dipole field vector B(u, i) = beta * [z_hat -
     3 (z_hat . r_hat) r_hat] at every point of a dense grid of argument-
     of-latitude u in [0, 2*pi) and orbit inclination i, from the standard
     orbital-frame triad r_hat(u,i), e_t(u,i), e_n(i) (radial, along-
     track, orbit-normal -- the same geometry wp13-edt-derivation.md
     Section 2.2 documents, re-derived independently here rather than
     copied).
  2. Numerically project B onto the triad to get B_r, B_t, B_n at every
     grid point (NOT assuming B_n is u-independent in advance -- that
     falls out of the numeric average, it is not baked into the code).
  3. Numerically compute the motional EMF (v x B).e_r and the Lorentz
     along-track drag force (I L (e_r x B)).e_t at every grid point, and
     confirm both reduce, at every u, to the same B_n factor (an internal
     consistency check on the geometry, Section 5 of the derivation doc).
  4. Orbit-average (trapezoidal quadrature over u) |B_n| and B_n^2 for a
     grid of inclinations i = 0 .. 98.4 deg, and form
         eta_F(i)  = avg|B_n(i)| / avg|B_n(0)|      (fixed-current model)
         eta_SC(i) = avg(B_n(i)^2) / avg(B_n(0)^2)  (self-consistent model)
     and compare against the closed forms cos(i) and cos(i)^2, and against
     the specific numeric table in wp13-edt-derivation.md Section 8.3.

Scope: this validates the ONE claim "eta(i) = |cos i| / cos^2 i for an
aligned dipole and a radial tether" against first-principles vector
projection + numerical quadrature. It does NOT validate: the tilted-
dipole caveat (Section 6, deliberately out of scope in both places), the
absolute B0/RE constants (universal reference values, not re-derived),
libration/T7, or any of the deorbit-TIME integration in edt_deorbit_years
(only the eta(i) EFFICIENCY FACTOR is checked here).

ASCII only, deterministic (no RNG).
"""
from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np

# Same cited constants as decay.cpp (SPENVIS/BIRA-IASB dipole reference).
# Universal reference values -- not something to re-derive independently.
B0 = 3.01153e-5             # T, equatorial surface field
RE_DIPOLE = 6371200.0       # m, dipole reference (mean) Earth radius
MU_EARTH = 3.986004418e14   # m^3/s^2

N_U = 4001  # dense uniform grid over one orbit, trapezoidal quadrature


def triad(u: np.ndarray, incl_rad: float):
    """Standard orbital-frame triad at argument of latitude u, inclination
    i (RAAN dropped -- it rotates everything about z and cancels out of
    orbit averages, as wp13-edt-derivation.md Section 2.2 notes; verified
    here only in the sense that our numeric average never uses RAAN)."""
    ci, si = math.cos(incl_rad), math.sin(incl_rad)
    cu, su = np.cos(u), np.sin(u)
    r_hat = np.stack([cu, su * ci, su * si], axis=-1)
    e_t = np.stack([-su, cu * ci, cu * si], axis=-1)
    e_n = np.array([0.0, -si, ci])
    return r_hat, e_t, e_n


def dipole_B(u: np.ndarray, incl_rad: float, r_m: float) -> np.ndarray:
    """Full 3D aligned-dipole field vector (ECI-like components, orbital
    frame with ascending node on x-axis), independent of any B_n
    shortcut."""
    beta = B0 * (RE_DIPOLE / r_m) ** 3
    r_hat, _e_t, _e_n = triad(u, incl_rad)
    z_hat = np.array([0.0, 0.0, 1.0])
    z_dot_r = r_hat @ z_hat  # (N,)
    B = beta * (z_hat[None, :] - 3.0 * z_dot_r[:, None] * r_hat)
    return B


def project_components(u, incl_rad, r_m):
    r_hat, e_t, e_n = triad(u, incl_rad)
    B = dipole_B(u, incl_rad, r_m)
    B_r = np.einsum("ij,ij->i", B, r_hat)
    B_t = np.einsum("ij,ij->i", B, e_t)
    B_n = B @ e_n
    return B_r, B_t, B_n, B, r_hat, e_t, e_n


def emf_and_force_consistency(u, incl_rad, r_m, v_mag, I_amp, L_m):
    """Independently compute (v x B).e_r (motional EMF) and
    I L (e_r x B).e_t (along-track Lorentz drag), and check both reduce
    to the SAME B_n factor at every sample point -- Section 5 of the
    derivation doc, verified numerically rather than assumed."""
    B_r, B_t, B_n, B, r_hat, e_t, e_n = project_components(u, incl_rad, r_m)
    v = v_mag * e_t  # velocity along-track
    v_cross_B = np.cross(v, B)
    Em = np.einsum("ij,ij->i", v_cross_B, r_hat)  # motional EMF (V/m)
    F = I_amp * L_m * np.cross(r_hat, B)
    Ft = np.einsum("ij,ij->i", F, e_t)            # along-track force (N)
    # Both SHOULD equal v*B_n and -I*L*B_n respectively at every u.
    em_residual = np.max(np.abs(Em - v_mag * B_n))
    ft_residual = np.max(np.abs(Ft + I_amp * L_m * B_n))
    return em_residual, ft_residual, B_n


def orbit_avg_eta(incl_deg_grid, r_m):
    u = np.linspace(0.0, 2.0 * math.pi, N_U, endpoint=False)
    du = u[1] - u[0]

    def avg(f):
        # Trapezoidal quadrature on a uniform periodic grid == simple mean
        # (endpoints coincide); stated explicitly rather than relying on
        # np.trapz's open-interval handling.
        return float(np.sum(f) * du / (2.0 * math.pi))

    _, _, Bn0, *_ = project_components(u, 0.0, r_m)
    ref_abs = avg(np.abs(Bn0))
    ref_sq = avg(Bn0 ** 2)

    eta_F, eta_SC, Bn_maxabs_over_u = [], [], []
    for incl_deg in incl_deg_grid:
        incl_rad = math.radians(incl_deg)
        _, _, Bn, *_ = project_components(u, incl_rad, r_m)
        eta_F.append(avg(np.abs(Bn)) / ref_abs)
        eta_SC.append(avg(Bn ** 2) / ref_sq)
        Bn_maxabs_over_u.append(float(np.max(Bn) - np.min(Bn)))
    return np.array(eta_F), np.array(eta_SC), np.array(Bn_maxabs_over_u)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    r_m = RE_DIPOLE + 800e3  # reference altitude for the field magnitude;
                             # eta is a ratio and is independent of r_m --
                             # verified below by repeating at a second r_m.

    print("=" * 78)
    print("Internal consistency: EMF and Lorentz-force projections both")
    print("reduce to the same B_n factor at every orbital phase u")
    print("(numerically checked, not assumed)")
    print("=" * 78)
    u_check = np.linspace(0.0, 2.0 * math.pi, N_U, endpoint=False)
    for incl_deg in (0.0, 30.0, 51.6, 71.0, 74.0, 90.0, 98.4):
        em_res, ft_res, Bn = emf_and_force_consistency(
            u_check, math.radians(incl_deg), r_m, v_mag=7500.0, I_amp=2.0,
            L_m=3000.0)
        bn_range = float(np.max(Bn) - np.min(Bn))
        print(f"  i={incl_deg:6.2f} deg: max|Em - v*Bn| = {em_res:.3e} V/m, "
              f"max|Ft + I*L*Bn| = {ft_res:.3e} N, "
              f"(max-min) of B_n over u = {bn_range:.3e} T "
              f"(0 => B_n truly u-independent, confirms Section 2.3's "
              f"analytic constancy numerically for this aligned dipole)")

    # ------------------------------------------------------------------
    # Main eta(i) verification.
    # ------------------------------------------------------------------
    print()
    print("=" * 78)
    print("eta(i) verification: numeric orbit-average vs closed form")
    print("=" * 78)

    table_incls = [0.0, 30.0, 51.6, 71.0, 74.0, 90.0, 98.4]
    eta_F_num, eta_SC_num, bn_range = orbit_avg_eta(table_incls, r_m)
    rows = []
    max_abs_err_F = 0.0
    max_abs_err_SC = 0.0
    for i_deg, ef, esc, rng in zip(table_incls, eta_F_num, eta_SC_num, bn_range):
        cf = abs(math.cos(math.radians(i_deg)))
        csc = math.cos(math.radians(i_deg)) ** 2
        err_f = abs(ef - cf)
        err_sc = abs(esc - csc)
        max_abs_err_F = max(max_abs_err_F, err_f)
        max_abs_err_SC = max(max_abs_err_SC, err_sc)
        print(f"  i={i_deg:6.2f} deg | numeric eta_F={ef:.10f}  closed |cos i|="
              f"{cf:.10f}  err={err_f:.2e} | numeric eta_SC={esc:.10f}  "
              f"closed cos^2 i={csc:.10f}  err={err_sc:.2e}")
        rows.append([i_deg, ef, cf, err_f, esc, csc, err_sc])

    print(f"\nMax |numeric - closed form| over the table: "
          f"eta_F {max_abs_err_F:.3e}, eta_SC {max_abs_err_SC:.3e}")

    # Cross-check against wp13-edt-derivation.md Section 8.3's own printed
    # GL-64 table (documented as agreeing with the closed form to 2.2e-16
    # there); confirm our independently-coded quadrature reproduces those
    # same printed values.
    derivation_table = {
        0.0: (1.0000000000, 1.0000000000),
        30.0: (0.8660254038, 0.7500000000),
        51.6: (0.6211477803, 0.3858245649),
        71.0: (0.3255681545, 0.1059946232),
        74.0: (0.2756373558, 0.0759759519),
        90.0: (0.0000000000, 0.0000000000),
    }
    print("\nCross-check vs wp13-edt-derivation.md Section 8.3 printed table:")
    max_err_vs_doc = 0.0
    for i_deg, ef, esc in zip(table_incls, eta_F_num, eta_SC_num):
        if i_deg not in derivation_table:
            continue
        doc_ef, doc_esc = derivation_table[i_deg]
        e1, e2 = abs(ef - doc_ef), abs(esc - doc_esc)
        max_err_vs_doc = max(max_err_vs_doc, e1, e2)
        print(f"  i={i_deg:6.2f} deg: eta_F num={ef:.10f} doc={doc_ef:.10f} "
              f"err={e1:.2e} | eta_SC num={esc:.10f} doc={doc_esc:.10f} "
              f"err={e2:.2e}")
    print(f"Max |numeric - documented table| = {max_err_vs_doc:.3e}")

    # Dense sweep (fine-grained, 0..98.4 deg) for monotonicity + limit
    # checks (L1/L2/L3 of wp13-edt-derivation.md Section 6), independently
    # re-verified rather than assumed.
    dense_incls = np.linspace(0.0, 98.4, 985)
    eta_F_dense, eta_SC_dense, _ = orbit_avg_eta(dense_incls, r_m)
    monotone_F = bool(np.all(np.diff(eta_F_dense[dense_incls <= 90.0]) <= 1e-9))
    monotone_SC = bool(np.all(np.diff(eta_SC_dense[dense_incls <= 90.0]) <= 1e-9))
    print(f"\nL1 (eta(0)=1): eta_F(0)={eta_F_dense[0]:.12f}, "
          f"eta_SC(0)={eta_SC_dense[0]:.12f}")
    idx90 = int(np.argmin(np.abs(dense_incls - 90.0)))
    print(f"L2 (monotone decreasing on [0,90] deg): eta_F {monotone_F}, "
          f"eta_SC {monotone_SC}")
    print(f"L3 (eta(90)=0, aligned-dipole exact zero): eta_F(90)="
          f"{eta_F_dense[idx90]:.3e}, eta_SC(90)={eta_SC_dense[idx90]:.3e}")
    idx984 = int(np.argmin(np.abs(dense_incls - 98.4)))
    print(f"i=98.4 deg (catalog C, sun-synchronous, retrograde): "
          f"eta_F={eta_F_dense[idx984]:.10f} (closed |cos 98.4|="
          f"{abs(math.cos(math.radians(98.4))):.10f}), "
          f"eta_SC={eta_SC_dense[idx984]:.10f} (closed cos^2 98.4="
          f"{math.cos(math.radians(98.4)) ** 2:.10f})")

    # r_m-independence check (eta is a ratio; verify it really is
    # independent of the reference altitude used for beta).
    r_m2 = RE_DIPOLE + 300e3
    eta_F_2, eta_SC_2, _ = orbit_avg_eta([71.0], r_m2)
    print(f"\nr_m-independence check at i=71 deg: eta_F(800km-ref)="
          f"{eta_F_num[table_incls.index(71.0)]:.12f} vs "
          f"eta_F(300km-ref)={eta_F_2[0]:.12f} "
          f"(diff {abs(eta_F_num[table_incls.index(71.0)] - eta_F_2[0]):.2e})")

    with open(os.path.join(here, "edt_eta_results.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["incl_deg", "eta_F_numeric", "eta_F_closed_abscos",
                    "eta_F_err", "eta_SC_numeric", "eta_SC_closed_cos2",
                    "eta_SC_err"])
        w.writerows(rows)
    print(f"\nWrote {os.path.join(here, 'edt_eta_results.csv')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
