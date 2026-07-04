#pragma once

#include "adsc/relmotion.hpp"

namespace adsc {

// ============================================================================
// Fidelity ladder: runtime-selectable relative-motion levels (WP12)
// ----------------------------------------------------------------------------
// F2 ("Passive-safety claims are model-scoped") states the WP1 keep-out and
// safety-ellipse guarantees are exact only in the linear Clohessy-Wiltshire
// (CW) model: circular target orbit, no J2, no differential drag, small
// separations. This module extends (never replaces, R1) that model with two
// higher levels that RE-VERIFY the same abort/coast geometry against a
// nonlinear, perturbed truth, without touching a single existing CW code path:
//
//   L0_CW       -- the ORIGINAL Clohessy-Wiltshire linearization (relmotion.hpp
//                  / CwModel::propagate_rk4). Every existing coast in the
//                  repo (WP1 corridor sweep, F1 abort verification, WP11
//                  guidance) already uses exactly this path; fidelity_coast_
//                  min_range's L0_CW branch calls it VERBATIM, so nothing that
//                  already exists changes by one bit (R1).
//   L1_J2       -- inertial two-body + J2 RK4 propagation of BOTH the target
//                  and the chaser in ECI, differenced into the target's
//                  instantaneous (rotating) LVLH frame at every sample step.
//   L2_J2_DRAG  -- L1_J2 plus a per-craft free-molecular drag term (reusing
//                  decay.cpp's atmospheric_density, R1).
//
// This is a single RUNTIME-SELECTABLE code path (one build, one binary,
// `level` is an ordinary function argument), which is strictly STRONGER than a
// build-time fork: every level is exercised by the same compiled program and
// the same ctest binary (see tests/test_ladder.cpp), so there is no risk of a
// level silently rotting behind an unbuilt `#ifdef`.
//
// ---- Target-orbit initialization (deterministic, R6) ----
// The target is initialized on a circular orbit at the catalog altitude and
// inclination (decay.cpp: SL-16/Zenit-2 840 km / 71 deg, SL-8/Kosmos-3M
// 750 km / 78 deg), RAAN = 0, argument of latitude u = 0 at t = 0. With the
// ascending node placed on the ECI x-axis (RAAN = 0) and orbital angular
// momentum direction h_hat = (0, -sin i, cos i) (standard result: rotate the
// equatorial pole by i about the node line), the position/velocity at
// argument of latitude u are, for a scalar radius a and circular speed
// v = sqrt(mu/a):
//     r(u) = a  * (cos u,        sin u * cos i,  sin u * sin i)
//     v(u) = a n * (-sin u,      cos u * cos i,  cos u * sin i)
// This is the t = 0 INITIAL CONDITION only; thereafter both craft are
// propagated by full nonlinear RK4 (two-body + J2 [+ drag]), not by this
// closed form.
//
// ---- The ONE documented LVLH transformation (R8) ----
// At every sample step the target's CURRENT (freshly propagated) ECI state
// (r_t, v_t) defines an orthonormal triad matching relmotion.hpp's convention
// exactly (x = radial, y = along-track, z = cross-track):
//     xhat = r_t / |r_t|
//     zhat = (r_t x v_t) / |r_t x v_t|      (orbit-normal / cross-track)
//     yhat = zhat x xhat                    (along-track: = h_hat x r_hat,
//                                             the exact velocity direction
//                                             for any r_t perp v_t decomp.)
// and the frame's own rotation rate (about zhat) is the exact osculating
// in-plane rate omega = (h/r^2) zhat, h = |r_t x v_t|, r = |r_t|. This omega
// is an EXACT identity for d(xhat)/dt (= omega x xhat) regardless of any
// perturbation: it follows purely from v_t = dr_t/dt and the r_t/v_t plane
// decomposition, not from an assumption of Keplerian motion.
//
// What IS approximated: under J2 the orbital plane itself slowly precesses
// (the physical origin of nodal/RAAN regression), so zhat's direction has a
// small additional drift not captured by "pure rotation about the current
// zhat". This CONVERSION uses the exact xhat/yhat/zhat/omega above only ONCE,
// to translate the caller's given LVLH relative velocity into an ECI initial
// condition for the chaser at t = 0 (r_chaser = r_t + C^T r_rel, v_chaser =
// v_t + C^T (v_rel + omega x r_rel), C = [xhat; yhat; zhat]^T rows). The
// neglected term is bounded by the nodal-regression rate itself,
// ~1.5 n J2 (Re/a)^2 |cos i| -- several orders of magnitude below the orbital
// rate n at LEO -- and it is applied EXACTLY ONCE: every subsequent sample
// step differences the ACTUAL propagated target/chaser ECI states through a
// FRESH xhat/yhat/zhat recomputed from the current (fully nonlinear) target
// state, never by advancing a fixed rotating frame, so there is no
// accumulating frame error over the coast -- only this one-shot IC bias,
// which manifests as a bounded velocity offset of order (nodal rate / n) *
// |v_rel| and hence a slow, sub-meter secular drift over a single orbit at
// the ranges this ladder uses (consistent with the design's own ~10 m-order
// J2 and ~1 m-order drag expectations).
//
// ---- Physical constants (cited, not PLACEHOLDER) ----
constexpr double kEarthJ2    = 1.08262668e-3;  // Earth J2 (oblateness), dimensionless
constexpr double kEarthOmega = 7.2921159e-5;   // Earth sidereal rotation rate [rad/s]

enum class FidelityLevel { L0_CW, L1_J2, L2_J2_DRAG };

// Inertial-differencing implementation, parameterized DIRECTLY on which
// perturbation terms are enabled (rather than the level enum) so the
// MANDATORY test_ladder.cpp cross-validation can isolate exactly the J2 and
// drag terms: L1 with J2 forced off must reproduce the L0 CW closed form
// (eps 0.5 m), and L2 with drag forced off must reproduce L1 bit-for-bit
// (same code path, same arguments). fidelity_coast_min_range (below) is a
// thin dispatcher on top of this for L1_J2/L2_J2_DRAG; L0_CW never calls it
// at all (bit-identical original CW path, see above).
//
//   x_rel_lvlh6   -- initial relative state [x,y,z,vx,vy,vz] in the target's
//                    LVLH frame at t = 0 (SAME convention as relmotion.hpp).
//   alt_km/incl_deg -- target circular-orbit altitude/inclination (see the
//                    target-orbit initialization above; RAAN = 0, u = 0).
//   horizon_s/dt_s -- propagation horizon and per-step RK4 dt (per-step
//                    minimum-range tracking, dt fixed by the caller -- SAME
//                    step pattern as every existing CW coast sweep in the
//                    repo, e.g. tests/test_guidance.cpp's fine_rk4_min_range).
//   solar_factor  -- atmospheric-density solar-activity factor (decay.cpp,
//                    T4); inert when enable_drag is false.
//   bc_chaser/bc_target -- per-craft ballistic coefficient Cd*(A/m) [m^2/kg]
//                    (matches decay.cpp's cd*area_over_mass product); inert
//                    when enable_drag is false.
double fidelity_coast_min_range_terms(bool enable_j2, bool enable_drag,
                                      const Vector6d& x_rel_lvlh6,
                                      double alt_km, double incl_deg,
                                      double horizon_s, double dt_s,
                                      double solar_factor,
                                      double bc_chaser, double bc_target);

// Runtime-selectable fidelity level (R1: one code path, not a build fork --
// see file header). L0_CW takes the ORIGINAL CW RK4 sweep verbatim (bit-
// identical to every existing CW coast in the repo); L1_J2/L2_J2_DRAG take
// the inertial ECI differencing path above (fidelity_coast_min_range_terms
// with enable_j2 = true, enable_drag = (level == L2_J2_DRAG)).
double fidelity_coast_min_range(FidelityLevel level, const Vector6d& x_rel_lvlh6,
                                double alt_km, double incl_deg,
                                double horizon_s, double dt_s, double solar_factor,
                                double bc_chaser, double bc_target);

}  // namespace adsc
