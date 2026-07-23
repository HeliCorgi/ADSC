# WP16 Digital Twin Phase 1 CSV schema (version 1.0)

[DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]. NO real asset
exists: `wp16_twin.csv` is emitted by `adsc_twin` (src/main_twin.cpp) from
a purely simulated lumped-mass EDT tether model (include/adsc/tether.hpp)
and a twin-to-twin sync demo (include/adsc/twin.hpp) in which a
perturbed-parameter simulated "truth" twin is assimilated by a reduced-
model EKF that never sees the truth parameters. T7 (EDT libration
dynamic-stability trade, _tasks_local/t7-libration-study.md) stays OPEN;
nothing here resolves it. The two controllers (C1 phase-gated, C2 fixed-
duty) are in-model PROPOSALS, evaluated only against this simulated
physics.

## Columns

| column | meaning |
|---|---|
| schema_version | WP16 schema id (`1.0`) |
| record_type | `dumbbell_validation` / `controller_comparison` / `twin_sync` |
| controller | `constant` / `c1_phase_gated` / `c2_fixed_duty` / `n/a` |
| metric | see record_type sections below |
| estimate | rate fraction, distribution mean, or a point value (see metric) |
| wilson_low, wilson_high | rate rows only: Wilson 95% CI |
| p05, p50, p95 | distribution rows only: 5th/50th/95th percentile |
| units | deg / orbit / fraction / - |
| notes | provenance, T7 cross-reference, PASS/FAIL band, or caveat |

## `dumbbell_validation` (Deliverable 4, target T4a ONLY)

N=2 rigid, massless-tether dumbbell (matches the T7 rigid-dumbbell exactly:
m1=9000 kg, m2=20 kg, L=3000 m, no dashpot). Three eps cases from T7 Table
5.1 (t7-libration-study.md Sec 5.1): eps=0.106 (BOUNDED, max_angle_deg band
[3,20] deg -- model-consistent with test_tether.cpp CHECK 3's own loose,
order-of-magnitude band, T7=9.26 deg), eps=0.49 and eps=0.80 (TUMBLE,
o45_orbit bands [0.15,0.30] and [0.10,0.20] orbit, T7=0.20/0.14).
`o45_orbit` uses the sentinel -1.0 for "never crossed within sim_orbits"
(never triggered for these three cases). notes carries the T7 reference
value, a PASS/FAIL verdict against the pre-registered band, and (eps=0.106
only) an explicit model-family-offset note: this lumped-mass fixed-step
RK4 reimplementation gives ~15.5 deg vs T7's own 1-DOF adaptive-DOP853
9.26 deg -- both are BOUNDED, the offset is integrator/model-family, not a
disagreement about the physics. A fourth row, metric
`t4b_status`, records that T4b (the 3D pitch+roll pumping-onset target) is
NOT attempted here -- it requires the Phase-2 out-of-plane model; the
Phase-1 planar deliverable is validated by T4a alone (stated up front, no
overclaim).

## `controller_comparison` (Deliverables 5 and 7)

N=8 lumped-mass model, Monte Carlo over the Deliverable-7 dispersions
(tip mass, EA_design, damping, eta_I, inclination, theta0, C2 switch
phase); NOTE the design record's initial-libration-RATE dispersion axis
is NOT applied in this Phase-1 implementation (the initial condition
always has zero relative velocity, matching the T7 "rates=0" seed
convention) -- a stated scope gap, not silently dropped. Rows per
controller (`constant` = uncontrolled baseline, `c1_phase_gated`,
`c2_fixed_duty`): per-status rates `diverged_angle_rate`,
`diverged_velocity_rate`, `overstrain_rate`, `energy_spike_rate` (EVERY
non-Ok DivergeStatus counted, each with Wilson 95% CI, denominator
n_runs) plus the combined `nonconverged_rate` (sum of the four, Wilson
CI) -- a fix for a prior accounting gap where only DivergedAngle was
counted and other truncation causes were invisible in the reported rate.
`n_clean` (point row) is the count of runs completing the full
sim_orbits horizon at status==Ok; `max_angle_deg` (p05/50/95),
`o45_orbit` (p05/50/95, finite-crossing runs only) plus
`o45_never_crossed_rate` (Wilson CI, denominator n_clean) reported
SEPARATELY rather than silently excluded, `eta_lib_effective` (p05/50/95
-- the headline stability-per-unit-thrust trade: C2 gives 0.75 by
construction, C1's is COMPUTED), and `energy_drift_per_orbit` (p05/50/95,
a numerics sanity diagnostic, not a stability claim) are ALL computed
over CLEAN (status==Ok) runs ONLY -- truncated runs are excluded from
every one of these percentile pools, never silently mixed in.

## `twin_sync` (Deliverable 6 and 7)

N=2 reduced-equivalent truth twin (matches the EKF's own reduced pitch-
pendulum order), controller = C1 phase-gated (the headline
virtual-to-real-pushback demo: Real runs the schedule Virtual computed).
`converged_rate` (TWIN-CONVERGED = parameter relative error < 10% for both
I_eff and c_hat, and NIS inside the 95% chi-square(2) band [0.05,7.38],
for >=5 consecutive orbits; Wilson CI), `i_eff_rel_err` / `c_hat_rel_err`
(final relative error, p05/50/95), `theta_rmse_deg` (p05/50/95),
`median_nis` (p05/50/95, filter-consistency sanity), and
`orbits_to_converge` (p05/50/95 over CONVERGED runs only; a -1.0 point row
replaces it if zero runs converged).

OBSERVABILITY NOTE [DT-v1]: c_hat is WEAKLY OBSERVABLE from the angle and
tension measurements in this configuration -- this twin-to-twin demo
estimates I_eff ROBUSTLY (it enters the pitch dynamics as the Lorentz
torque and moves the angle innovation directly), while the effective pitch
damping c_hat is identifiable only as BOUNDED-WITH-HONEST-UNCERTAINTY. c_hat
is the EKF's tunable effective pitch-damping (gamma=c_hat/(2*mu)), a
DIFFERENT physical quantity from the truth twin's per-segment AXIAL dashpot
c_true, which produces ~zero direct pitch damping in near-rigid rotation
(the free-decay rate is ~10 orders below gamma; the tension channel that
does respond to c_true is not connected to c_hat by the measurement model).
So `c_hat_rel_err` (distance from c_true) is reported FOR THE RECORD but is
NOT expected to be small, and the `converged_rate`'s c_hat<10% clause is
correspondingly a stringent, mostly-informational gate -- see
_tasks_local/wp16_xcheck.py and the TwinSyncReport finding note
(include/adsc/twin.hpp). This is a genuine weak-observability result, not a
filter defect.

## Honesty footer (every table in `wp16_twin.md`)

T7 stays OPEN; C1/C2 are in-model PROPOSALS, not a resolved stability
mechanism; `eta_libration`=0.75 (C2's `duty_on`) is an average-thrust
bookkeeping factor, not a stability margin; the reduced-EKF and
planar-Phase-1 model are blind to the out-of-plane (roll) pumping channel
-- a stated limitation, not a safety proof.
