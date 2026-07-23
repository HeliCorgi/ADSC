# WP16 Digital Twin Phase 1 -- summary

[DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]

NO real asset exists. This is a TWIN-TO-TWIN exercise: a perturbed-parameter simulated "truth" assimilated by a reduced-model EKF that never sees the truth parameters. Monte Carlo sections use 200 runs per controller/case. Regenerate with `adsc_twin`.

## Dumbbell-limit validation (Deliverable 4, T4a)

| controller | metric | estimate | 95% CI / p05..p95 | units | notes |
|---|---|---:|---|---|---|
| constant | max_angle_deg | 15.4572 | - | deg | T4a eps=0.106 BOUNDED band [3.0,20.0] deg (T7=9.26 deg); PASS; lumped-mass RK4 gives ~15.5 deg vs the T7 1-DOF DOP853 9.26 deg - model-family offset, both bounded |
| constant | o45_orbit | 0.1916 | - | orbit | T4a eps=0.490 TUMBLE band o45 [0.15,0.30] orbit (T7=0.20); -1 = never; PASS |
| constant | o45_orbit | 0.1413 | - | orbit | T4a eps=0.800 TUMBLE band o45 [0.10,0.20] orbit (T7=0.14); -1 = never; PASS |
| n/a | t4b_status | 0.0000 | - | - | T4b (3D pitch+roll pumping onset, o45 in [0.40,0.70] orbit, 80-deg event by orbit [3.5,5.5]) requires the Phase-2 out-of-plane model and is NOT attempted in this Phase-1 planar implementation -- stated up front, no overclaim. |

## Controller comparison Monte Carlo (Deliverables 5, 7)

| controller | metric | estimate | 95% CI / p05..p95 | units | notes |
|---|---|---:|---|---|---|
| constant | diverged_angle_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::DivergedAngle (chord angle-from-vertical > 80 deg); Wilson 95% CI |
| constant | diverged_velocity_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::DivergedVelocity (any |v_i| > 10*n*L); Wilson 95% CI |
| constant | overstrain_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::Overstrain (any segment l_j/L0 > 1.5); Wilson 95% CI |
| constant | energy_spike_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::EnergySpike (recalibrated guard, tether.cpp -- genuine integrator blow-ups only); Wilson 95% CI |
| constant | nonconverged_rate | 0.0000 | [0.0000, 0.0188] | fraction | combined fraction hitting ANY non-Ok status (sum of the four rows above); every non-Ok run is EXCLUDED from the amplitude/o45/eta/energy pools below, never silently mixed in -- see n_clean; Wilson 95% CI |
| constant | n_clean | 200.0000 | - | count | runs completing the full sim_orbits horizon at status==Ok; the max_angle/o45/eta_lib/energy_drift rows below are computed over ONLY these n_clean runs (n_clean = n_runs - n_nonconverged) |
| constant | max_angle_deg | 10.1540 | 4.880 .. 9.264 .. 18.246 | deg | peak chord angle-from-vertical over the run; CLEAN (status==Ok, n_clean) runs only -- truncated runs excluded, no silent corruption |
| constant | o45_never_crossed_rate | 1.0000 | [0.9812, 1.0000] | fraction | fraction of CLEAN runs (denominator n_clean, NOT n_runs) that never reached 45 deg within sim_orbits; Wilson 95% CI |
| constant | eta_lib_effective | 0.7605 | 0.521 .. 0.771 .. 0.983 | fraction | time-average |I_applied|/I_cap actually delivered; CLEAN runs only; C2 gives duty_on=0.75 by construction, C1's is COMPUTED (the stability-per-unit-thrust headline) |
| constant | energy_drift_per_orbit | 0.0000 | 0.000 .. 0.000 .. 0.000 | fraction | last full-orbit |E_J drift - integrated (P_lorentz+P_damp)| / (mu*L^2*n^2); CLEAN runs only; sanity diagnostic, not a stability claim |
| c1_phase_gated | diverged_angle_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::DivergedAngle (chord angle-from-vertical > 80 deg); Wilson 95% CI |
| c1_phase_gated | diverged_velocity_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::DivergedVelocity (any |v_i| > 10*n*L); Wilson 95% CI |
| c1_phase_gated | overstrain_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::Overstrain (any segment l_j/L0 > 1.5); Wilson 95% CI |
| c1_phase_gated | energy_spike_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::EnergySpike (recalibrated guard, tether.cpp -- genuine integrator blow-ups only); Wilson 95% CI |
| c1_phase_gated | nonconverged_rate | 0.0000 | [0.0000, 0.0188] | fraction | combined fraction hitting ANY non-Ok status (sum of the four rows above); every non-Ok run is EXCLUDED from the amplitude/o45/eta/energy pools below, never silently mixed in -- see n_clean; Wilson 95% CI |
| c1_phase_gated | n_clean | 200.0000 | - | count | runs completing the full sim_orbits horizon at status==Ok; the max_angle/o45/eta_lib/energy_drift rows below are computed over ONLY these n_clean runs (n_clean = n_runs - n_nonconverged) |
| c1_phase_gated | max_angle_deg | 10.1729 | 4.880 .. 9.264 .. 18.246 | deg | peak chord angle-from-vertical over the run; CLEAN (status==Ok, n_clean) runs only -- truncated runs excluded, no silent corruption |
| c1_phase_gated | o45_never_crossed_rate | 1.0000 | [0.9812, 1.0000] | fraction | fraction of CLEAN runs (denominator n_clean, NOT n_runs) that never reached 45 deg within sim_orbits; Wilson 95% CI |
| c1_phase_gated | eta_lib_effective | 0.7602 | 0.521 .. 0.771 .. 0.983 | fraction | time-average |I_applied|/I_cap actually delivered; CLEAN runs only; C2 gives duty_on=0.75 by construction, C1's is COMPUTED (the stability-per-unit-thrust headline) |
| c1_phase_gated | energy_drift_per_orbit | 0.0000 | 0.000 .. 0.000 .. 0.000 | fraction | last full-orbit |E_J drift - integrated (P_lorentz+P_damp)| / (mu*L^2*n^2); CLEAN runs only; sanity diagnostic, not a stability claim |
| c2_fixed_duty | diverged_angle_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::DivergedAngle (chord angle-from-vertical > 80 deg); Wilson 95% CI |
| c2_fixed_duty | diverged_velocity_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::DivergedVelocity (any |v_i| > 10*n*L); Wilson 95% CI |
| c2_fixed_duty | overstrain_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction hitting DivergeStatus::Overstrain (any segment l_j/L0 > 1.5); Wilson 95% CI |
| c2_fixed_duty | energy_spike_rate | 0.1200 | [0.0820, 0.1723] | fraction | fraction hitting DivergeStatus::EnergySpike (recalibrated guard, tether.cpp -- genuine integrator blow-ups only); Wilson 95% CI |
| c2_fixed_duty | nonconverged_rate | 0.1200 | [0.0820, 0.1723] | fraction | combined fraction hitting ANY non-Ok status (sum of the four rows above); every non-Ok run is EXCLUDED from the amplitude/o45/eta/energy pools below, never silently mixed in -- see n_clean; Wilson 95% CI |
| c2_fixed_duty | n_clean | 176.0000 | - | count | runs completing the full sim_orbits horizon at status==Ok; the max_angle/o45/eta_lib/energy_drift rows below are computed over ONLY these n_clean runs (n_clean = n_runs - n_nonconverged) |
| c2_fixed_duty | max_angle_deg | 13.6183 | 6.454 .. 12.668 .. 22.818 | deg | peak chord angle-from-vertical over the run; CLEAN (status==Ok, n_clean) runs only -- truncated runs excluded, no silent corruption |
| c2_fixed_duty | o45_never_crossed_rate | 1.0000 | [0.9786, 1.0000] | fraction | fraction of CLEAN runs (denominator n_clean, NOT n_runs) that never reached 45 deg within sim_orbits; Wilson 95% CI |
| c2_fixed_duty | eta_lib_effective | 0.5574 | 0.389 .. 0.553 .. 0.726 | fraction | time-average |I_applied|/I_cap actually delivered; CLEAN runs only; C2 gives duty_on=0.75 by construction, C1's is COMPUTED (the stability-per-unit-thrust headline) |
| c2_fixed_duty | energy_drift_per_orbit | 0.0000 | 0.000 .. 0.000 .. 0.000 | fraction | last full-orbit |E_J drift - integrated (P_lorentz+P_damp)| / (mu*L^2*n^2); CLEAN runs only; sanity diagnostic, not a stability claim |

## Twin-to-twin sync Monte Carlo (Deliverables 6, 7)

| controller | metric | estimate | 95% CI / p05..p95 | units | notes |
|---|---|---:|---|---|---|
| c1_phase_gated | converged_rate | 0.0000 | [0.0000, 0.0188] | fraction | fraction TWIN-CONVERGED (param rel err < 10% and NIS in [0.05,7.38] for >=5 consecutive orbits); Wilson 95% CI |
| c1_phase_gated | i_eff_rel_err | 0.1709 | 0.016 .. 0.171 .. 0.345 | fraction | final |I_eff_hat - I_eff_true| / I_eff_true |
| c1_phase_gated | c_hat_rel_err | 1.0068 | 0.082 .. 0.504 .. 3.148 | fraction | final |c_hat - c_true| / c_true; c_hat is WEAKLY OBSERVABLE [DT-v1] (effective pitch damping, NOT the axial c_true) so this is reported for the record, NOT expected small -- I_eff is the robustly-identified parameter; see twin.hpp TwinSyncReport finding note + _tasks_local/wp16_xcheck.py |
| c1_phase_gated | theta_rmse_deg | 0.0677 | 0.053 .. 0.066 .. 0.083 | deg | RMSE of (EKF theta estimate - truth chord angle) over the whole run |
| c1_phase_gated | median_nis | 2.7586 | 2.746 .. 2.758 .. 2.771 | - | per-run median 2-dof NIS; 95% chi-square(2) band is [0.05, 7.38] |
| c1_phase_gated | orbits_to_converge | -1.0000 | - | orbit | no run converged within sim_orbits; -1 sentinel |

## Honesty footer

T7 (EDT libration dynamic-stability trade) stays OPEN; C1/C2 are in-model PROPOSALS, not a resolved stability mechanism; eta_libration=0.75 (C2's duty_on) is an average-thrust bookkeeping factor, not a stability margin; the reduced-EKF and planar-Phase-1 model are blind to the out-of-plane (roll) pumping channel -- a stated limitation, not a safety proof.
