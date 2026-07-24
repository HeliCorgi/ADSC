// WP16 twin-to-twin tests (Deliverable 6): EKF covariance stays symmetric
// and positive-definite over a long run, parameter estimates move toward
// (not away from) the truth on a fixed seed, and determinism (two runs
// bit-identical). Explicit return-1 checks (R4).
//
// [DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]. NO real asset:
// "truth" here is itself a perturbed-parameter SIMULATED instance of the
// tether.hpp model (see twin.hpp's file header). T7 stays OPEN; nothing
// here claims otherwise.
#include <cmath>
#include <cstdio>

#include <Eigen/Dense>

#include "adsc/tether.hpp"
#include "adsc/twin.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

namespace {

TruthTwinConfig make_truth(double ea_true, double c_true, double eta_i_true) {
    TetherConfig truth;
    truth.n_beads = 2;
    truth.tether_length_m = 3000.0;
    truth.m_parent_kg = 9000.0;
    truth.m_tip_kg = 20.0;
    truth.lambda_tether_kg_per_m = 0.0;
    truth.altitude_km = 840.0;
    truth.inclination_deg = 71.0;
    truth.EA_design_N = ea_true;
    truth.damping_c_Ns_per_m = c_true;
    truth.eta_I = eta_i_true;
    truth.I_cap_A = 2.0;
    truth.theta0_deg = 3.0;
    truth.dt_s = 0.2;
    truth.const_current_a = truth.eta_I * truth.I_cap_A;

    TruthTwinConfig tc;
    tc.truth_tether = truth;
    tc.sigma_theta_deg = 0.5;
    tc.sigma_tension_n = 0.05;
    return tc;
}

VirtualTwinConfig make_virtual() {
    VirtualTwinConfig vcfg;
    vcfg.altitude_km = 840.0;
    vcfg.inclination_deg = 71.0;
    vcfg.m_parent_kg = 9000.0;
    vcfg.m_tip_kg = 20.0;
    vcfg.tether_length_m = 3000.0;
    return vcfg;
}

}  // namespace

int main() {
    // 1. Covariance stays symmetric and positive-definite over a long run
    //    of predict()/update() cycles fed FIXED synthetic measurements
    //    (isolating the EKF machinery itself from the truth-twin/sensor
    //    plumbing exercised by tests 2-3).
    {
        VirtualTwinConfig vcfg = make_virtual();
        EkfState x0;
        x0.x = Eigen::Vector4d(3.0 * kPi / 180.0, 0.0, 1.0, 0.05);
        x0.P = Eigen::Matrix4d::Identity() * 0.01;
        VirtualTwinEkf ekf(vcfg, x0);

        const double dt = 0.2;
        for (int i = 0; i < 500; ++i) {
            ekf.predict(dt);
            // Fixed synthetic measurements (not from any truth sim): a
            // constant small angle and a plausible tension value. The point
            // of this test is the FILTER's own numerical discipline, not
            // whether these particular numbers are physically consistent.
            const double nis = ekf.update(3.0 * kPi / 180.0, 0.25, 0.5 * kPi / 180.0, 0.05);
            CHECK(std::isfinite(nis));
            CHECK(nis >= 0.0);

            const Eigen::Matrix4d& P = ekf.state().P;
            CHECK((P - P.transpose()).cwiseAbs().maxCoeff() < 1e-8);
            const Eigen::LLT<Eigen::Matrix4d> llt(P);
            CHECK(llt.info() == Eigen::Success);  // positive-definite (Cholesky succeeds)
        }
    }

    // 2. The estimator recovers the OBSERVABLE quantities and reports the
    //    WEAKLY-observable one honestly, on a fixed seed.
    //
    //    WEAK-OBSERVABILITY FINDING [DT-v1] (this block was restructured after
    //    an earlier version wrongly asserted that c_hat converges to c_true):
    //    I_eff and c_hat are NOT equally observable from the (angle, tension)
    //    measurements here, and c_hat is NOT the same physical quantity as the
    //    truth twin's axial dashpot c_true. See the finding note on
    //    TwinSyncReport (include/adsc/twin.hpp) and VirtualTwinConfig::q_c_hat:
    //    c_hat is the EKF's TUNABLE effective pitch-damping (gamma=c_hat/(2*mu)),
    //    while c_true is a per-segment AXIAL dashpot that produces ~zero direct
    //    pitch damping in near-rigid rotation. An independent finite-difference
    //    cross-check (_tasks_local/wp16_xcheck.py, part (d) + the innovation-
    //    sensitivity extension) confirms it two ways: (i) the truth twin's
    //    free-decay rate is ~10 orders of magnitude below gamma, and (ii) the
    //    angle measurement is ~2.7e6x more sensitive (noise-normalized) to
    //    i_eff_true than to c_true, while the tension channel that DOES respond
    //    to c_true is not connected to c_hat by the measurement model (H(1,3)=0).
    //    So the (angle,tension) data carry essentially no information about
    //    c_true, and there is NO data-driven reason for c_hat to converge to it
    //    -- doing so would be luck, not correctness. The asserts below therefore:
    //      - PIN the strongly-observable I_eff (it must improve on the generic
    //        prior AND land in a tight band),
    //      - require good angle tracking, covariance symmetric+PD throughout,
    //        and a filter-consistent NIS,
    //      - and require c_hat to stay FINITE and BOUNDED with its variance NOT
    //        collapsing to spurious certainty (the filter must KNOW it doesn't
    //        know), instead of the old (deleted) c_hat==c_true equivalence.
    //    Numeric bounds are honest, replica-derived slack bounds (a from-scratch
    //    Python replica of this exact config gives I_eff_rel_err ~0.16-0.21 --
    //    hence NOT a <0.15 pin -- theta_rmse ~0.05-0.07 deg, min P(3,3) ~9e-4,
    //    |c_hat| <= 0.13; wp16_xcheck.py), not tuned-to-green pins.
    {
        const double ea_true = 12000.0, c_true = 0.06, eta_i_true = 0.8;
        const TruthTwinConfig tc = make_truth(ea_true, c_true, eta_i_true);
        const VirtualTwinConfig vcfg = make_virtual();

        const double i_eff_true = eta_i_true * tc.truth_tether.I_cap_A;
        const double i_eff_guess0 = 1.0;  // matches run_twin_sync's own PLACEHOLDER prior
        const double c_hat_guess0 = 0.05;  // matches run_twin_sync's own PLACEHOLDER prior
        const double i_err0 = std::fabs(i_eff_guess0 - i_eff_true) / i_eff_true;

        const TwinSyncReport rep = run_twin_sync(tc, vcfg, ControllerMode::PhaseGated, 15.0, 42ULL);

        // --- strongly-observable I_eff: it must genuinely converge ---
        CHECK(std::isfinite(rep.final_I_eff_rel_err));
        CHECK(rep.final_I_eff_rel_err < i_err0);   // improved on the generic prior (real observability signal)
        CHECK(rep.final_I_eff_rel_err < 0.30);     // and landed in a tight band (replica ~0.16-0.21)

        // --- angle tracking + filter consistency ---
        CHECK(std::isfinite(rep.theta_rmse_deg) && rep.theta_rmse_deg >= 0.0);
        CHECK(rep.theta_rmse_deg < 2.0);           // filter tracks the strongly-observed angle (replica ~0.05-0.07 deg)
        CHECK(std::isfinite(rep.median_nis) && rep.median_nis >= 0.0);

        // --- covariance stays symmetric AND positive-definite for the whole run ---
        CHECK(rep.cov_spd_all_steps);

        // --- weakly-observable c_hat: FINITE, BOUNDED, and NOT spuriously certain ---
        CHECK(std::isfinite(rep.final_c_hat));
        CHECK(std::isfinite(rep.final_c_hat_rel_err));
        CHECK(std::fabs(rep.final_c_hat) < 10.0 * c_hat_guess0);  // within 10x the prior guess (0.5); never runs away (replica |c_hat| <= 0.13)
        CHECK(rep.min_c_hat_variance > 1e-4);      // variance never collapses -> the filter reports it does NOT know c_hat (replica min ~9e-4)
        CHECK(std::isfinite(rep.min_c_hat_variance));
    }

    // 3. Determinism: two runs of the identical case are bit-identical (R6).
    {
        const TruthTwinConfig tc = make_truth(9000.0, 0.04, 0.65);
        const VirtualTwinConfig vcfg = make_virtual();
        const TwinSyncReport r1 = run_twin_sync(tc, vcfg, ControllerMode::FixedDuty, 5.0, 777ULL);
        const TwinSyncReport r2 = run_twin_sync(tc, vcfg, ControllerMode::FixedDuty, 5.0, 777ULL);
        CHECK(r1.converged == r2.converged);
        CHECK(r1.converged_at_orbit == r2.converged_at_orbit);
        CHECK(r1.final_I_eff_rel_err == r2.final_I_eff_rel_err);
        CHECK(r1.final_c_hat_rel_err == r2.final_c_hat_rel_err);
        CHECK(r1.theta_rmse_deg == r2.theta_rmse_deg);
        CHECK(r1.median_nis == r2.median_nis);
    }

    // 4. [DT-v2: 3D] VirtualTwinEkf3D (Deliverable D8): covariance health
    //    over a long run of predict()/update() cycles (mirroring test 1's
    //    isolation of the filter's own numerical discipline from the
    //    truth-twin/sensor plumbing), PLUS the pre-registered OBSERVABILITY
    //    FINDING demonstrated directly: a scalar cone-angle-only sensor
    //    cannot shrink the roll (phi) state's variance, while the
    //    recommended 2-axis chord-direction sensor does.
    {
        VirtualTwinConfig3D vcfg;
        vcfg.altitude_km = 840.0;
        vcfg.inclination_deg = 71.0;
        vcfg.m_parent_kg = 9000.0;
        vcfg.m_tip_kg = 20.0;
        vcfg.tether_length_m = 3000.0;

        EkfState3D x0;
        // Comma-initializer (not a 6-arg constructor call): portable across
        // Eigen 3.3+ for a fixed-size 6x1 matrix.
        x0.x << 3.0 * kPi / 180.0, 0.0, 3.0 * kPi / 180.0, 0.0, 1.0, 0.05;
        x0.P = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;

        const double dt = 0.2;
        const double n = tether_mean_motion_rad_s(vcfg.altitude_km);

        // 4a. Covariance stays symmetric AND positive-definite over 500
        //     predict()/update_chord2axis() cycles fed FIXED synthetic
        //     measurements (same house pattern as the 4-state test 1 above).
        {
            VirtualTwinEkf3D ekf(vcfg, x0);
            double u = 0.0;
            for (int i = 0; i < 500; ++i) {
                ekf.predict(dt, u);
                u += n * dt;
                const double nis = ekf.update_chord2axis(
                    std::sin(3.0 * kPi / 180.0), std::sin(3.0 * kPi / 180.0), 0.25,
                    0.5 * kPi / 180.0, 0.5 * kPi / 180.0, 0.05);
                CHECK(std::isfinite(nis));
                CHECK(nis >= 0.0);

                const Eigen::Matrix<double, 6, 6>& P = ekf.state().P;
                CHECK((P - P.transpose()).cwiseAbs().maxCoeff() < 1e-7);
                const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(P);
                CHECK(llt.info() == Eigen::Success);
            }
        }

        // 4b. OBSERVABILITY FINDING: run the SAME predict/measurement
        //     schedule (fixed synthetic (theta,phi)=3deg truth) through
        //     TWO independent filters that differ ONLY in which update()
        //     they call -- update_scalar_angle (cone angle only) vs
        //     update_chord2axis (2-axis chord + tension) -- and compare the
        //     roll (phi) variance P(2,2) after many cycles. Thresholds
        //     below are honest, replica-derived slack bounds (a from-
        //     scratch Python replica of this exact filter gives
        //     p22_scalar/p22_initial ~ 0.35 and p22_chord/p22_initial ~
        //     1.2e-4 after 300 cycles, i.e. p22_chord/p22_scalar ~ 3.4e-4
        //     -- NOT tuned-to-pass pins): the scalar-angle-only filter's
        //     roll variance shrinks only MODESTLY (a confounded, entangled
        //     signal it cannot cleanly attribute to phi alone), while the
        //     chord2axis filter's shrinks by several more orders of
        //     magnitude. This is the pre-registered [DT-v2: 3D]
        //     roll-observability finding (twin.hpp class comment),
        //     demonstrated directly via the COMPARATIVE ratio, not an
        //     absolute-floor claim on the scalar filter alone.
        {
            VirtualTwinEkf3D ekf_scalar(vcfg, x0);
            VirtualTwinEkf3D ekf_chord(vcfg, x0);
            const double true_ang = std::acos(std::cos(3.0 * kPi / 180.0) * std::cos(3.0 * kPi / 180.0));
            const double true_y = std::sin(3.0 * kPi / 180.0) * std::cos(3.0 * kPi / 180.0);
            const double true_z = std::sin(3.0 * kPi / 180.0);
            double u_s = 0.0, u_c = 0.0;
            for (int i = 0; i < 300; ++i) {
                ekf_scalar.predict(dt, u_s);
                u_s += n * dt;
                ekf_scalar.update_scalar_angle(true_ang, 0.5 * kPi / 180.0);

                ekf_chord.predict(dt, u_c);
                u_c += n * dt;
                ekf_chord.update_chord2axis(true_y, true_z, 0.25,
                                            0.5 * kPi / 180.0, 0.5 * kPi / 180.0, 0.05);
            }
            const double p22_scalar = ekf_scalar.state().P(2, 2);
            const double p22_chord = ekf_chord.state().P(2, 2);
            CHECK(std::isfinite(p22_scalar) && p22_scalar > 0.0);
            CHECK(std::isfinite(p22_chord) && p22_chord > 0.0);
            // scalar-angle-only: roll variance stays well above the
            // chord-sensor floor (replica ~3.5e-3, generous margin below).
            CHECK(p22_scalar > 1.0e-3);
            // chord2axis: roll variance DOES collapse substantially
            // (replica ~1.2e-6, generous margin above).
            CHECK(p22_chord < 1.0e-4);
            // THE comparative observability finding (replica ratio ~3.4e-4;
            // 0.01 threshold gives ~30x margin): a dedicated 2-axis
            // chord-direction sensor identifies roll far better than a
            // scalar angle-only sensor can.
            CHECK(p22_chord < 0.01 * p22_scalar);
        }
    }

    // 5. [DT-v2: 3D] DIRECTION-OF-FORCING check (adversarial-review
    //    follow-up). Test 4 above exercises covariance/variance behavior
    //    ONLY -- that trace is IDENTICAL whether nondim_torques' q_phi has
    //    the right sign or is sign-inverted (a pure sign flip in a linear
    //    forcing term changes no variance), so it could not have caught the
    //    q_phi sign inversion an adversarial review found and src/twin.cpp
    //    now fixes. This test targets the SIGN directly, cross-checked
    //    against the INDEPENDENT truth bead model (tether.cpp), not just
    //    the EKF's own internal consistency.
    //
    //    Setup: theta=phi=0 (dumbbell aligned along +x/radial), a PURE
    //    cross-track field B=(0,Bt,0) with Bt>0, and I=+1A.
    //
    //    Field realization note: tether.hpp exposes no direct field-
    //    injection hook (field_vector_hill is file-local to tether.cpp,
    //    derived only from altitude/inclination/argument-of-latitude u) --
    //    so B=(0,Bt,0) is realized via the SMALLEST orbit-position/
    //    inclination choice that gives it EXACTLY at t=0: a polar orbit
    //    (inclination=90deg) zeroes B_n=beta*cos(90deg) identically, and
    //    u=0 (t=0) zeroes B_r=-2*beta*sin(90deg)*sin(0) identically,
    //    leaving B_t=beta*sin(90deg)*cos(0)=beta>0 as the only nonzero
    //    component -- exactly the review's B=(0,Bt,0) scenario.
    //
    //    Expected physics (the review's own derivation): F=I*(e x B) with
    //    e=(1,0,0) (the +x tip direction) and B=(0,Bt,0) gives F=(0,0,Bt),
    //    a PURE +z force, so the +x tip must tilt toward +z, i.e. phi
    //    (roll) increases from 0. At this same operating point
    //    (theta=phi=thetadot=phidot=0), the 6-state EKF's phi_ddot
    //    (predict()'s f(), dx(3)) reduces to EXACTLY n^2*q_phi (every
    //    sin(phi)-carrying term vanishes), so it must carry the SAME sign
    //    -- an independent, cross-model check that a sign inversion in
    //    EITHER the forcing f() or the Jacobian's per-unit-current probe
    //    (F(3,4), which reuses the same nondim_torques call) cannot hide
    //    behind (both were verified against this same Python cross-check
    //    pre-commit; see the wp16-phase2-3d branch history).
    {
        // (a) TRUTH bead model (tether.cpp): the exact dumbbell config
        // above, constant current +1A, a few RK4 steps from rest.
        TetherConfig truth_cfg;
        truth_cfg.n_beads = 2;                  // rigid dumbbell limit
        truth_cfg.tether_length_m = 3000.0;
        truth_cfg.m_parent_kg = 9000.0;
        truth_cfg.m_tip_kg = 20.0;
        truth_cfg.lambda_tether_kg_per_m = 0.0;
        truth_cfg.altitude_km = 840.0;
        truth_cfg.inclination_deg = 90.0;       // polar orbit: B_n == 0 identically (see block comment)
        truth_cfg.out_of_plane = true;
        truth_cfg.theta0_deg = 0.0;
        truth_cfg.phi0_deg = 0.0;
        truth_cfg.controller = ControllerMode::Constant;
        truth_cfg.const_current_a = 1.0;        // I = +1 A
        truth_cfg.dt_s = 0.2;

        TetherSim truth(truth_cfg);
        CHECK(truth.roll_angle_rad() == 0.0);   // exact IC: phi0=0 (sin(0)==0.0 exactly)
        for (int i = 0; i < 10; ++i) {
            truth.step();
        }
        CHECK(truth.status() == DivergeStatus::Ok);  // sanity: this tiny run must not have diverged
        const double truth_roll_rad = truth.roll_angle_rad();
        CHECK(std::isfinite(truth_roll_rad));
        CHECK(truth_roll_rad > 0.0);  // phi increases from 0, per the review's F=(0,0,Bt) derivation

        // (b) EKF predict() at the SAME (theta=0,phi=0,thetadot=0,phidot=0)
        // operating point and the SAME field (altitude=840km matches the
        // truth beta; inclination=90deg + u_rad=0 realizes the same
        // B=(0,Bt,0)).
        VirtualTwinConfig3D vcfg;
        vcfg.altitude_km = 840.0;
        vcfg.inclination_deg = 90.0;
        vcfg.m_parent_kg = 9000.0;
        vcfg.m_tip_kg = 20.0;
        vcfg.tether_length_m = 3000.0;

        EkfState3D x0;
        x0.x << 0.0, 0.0, 0.0, 0.0, 1.0, 0.05;  // theta=thetadot=phi=phidot=0, I_eff=+1A, c_hat PLACEHOLDER prior
        x0.P = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        VirtualTwinEkf3D ekf(vcfg, x0);
        ekf.predict(0.2, 0.0);  // u_rad=0 (t=0), same dt as the truth step above

        // Since thetadot0==phidot0==0 exactly, x(3) (phi_dot) after this
        // ONE step is (to RK4 order) just dt*phi_ddot(0) -- its sign IS the
        // sign of d(phi_dot)/dt at the operating point.
        const double ekf_phidot_after_step = ekf.state().x(3);
        CHECK(std::isfinite(ekf_phidot_after_step));
        CHECK(ekf_phidot_after_step > 0.0);

        // --- THE direction-of-forcing check: truth and EKF must AGREE on
        // sign, and both must be positive (a sign-inverted q_phi would flip
        // ekf_phidot_after_step negative here -- verified directly against
        // this exact test setup before the fix landed). ---
        CHECK((truth_roll_rad > 0.0) == (ekf_phidot_after_step > 0.0));
        CHECK(truth_roll_rad > 0.0 && ekf_phidot_after_step > 0.0);
    }

    std::printf("twin: all tests passed\n");
    return 0;
}
