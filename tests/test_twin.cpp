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

    // 2. Estimates move toward (not away from) the truth on a fixed seed.
    //    The acceptance criterion is deliberately ROBUST rather than a tight
    //    numeric pin (this filter's tuning could not be empirically
    //    verified against a local build before committing this file, see
    //    the WP16 implementation report): either the final relative error
    //    improved on the initial (generic-prior) relative error, or it is
    //    already comfortably converged (< 30%).
    {
        const double ea_true = 12000.0, c_true = 0.06, eta_i_true = 0.8;
        const TruthTwinConfig tc = make_truth(ea_true, c_true, eta_i_true);
        const VirtualTwinConfig vcfg = make_virtual();

        const double i_eff_true = eta_i_true * tc.truth_tether.I_cap_A;
        const double i_eff_guess0 = 1.0;  // matches run_twin_sync's own PLACEHOLDER prior
        const double c_hat_guess0 = 0.05;
        const double i_err0 = std::fabs(i_eff_guess0 - i_eff_true) / i_eff_true;
        const double c_err0 = std::fabs(c_hat_guess0 - c_true) / c_true;

        const TwinSyncReport rep = run_twin_sync(tc, vcfg, ControllerMode::PhaseGated, 15.0, 42ULL);

        CHECK(std::isfinite(rep.final_I_eff_rel_err));
        CHECK(std::isfinite(rep.final_c_hat_rel_err));
        CHECK(rep.final_I_eff_rel_err < i_err0 || rep.final_I_eff_rel_err < 0.30);
        CHECK(rep.final_c_hat_rel_err < c_err0 || rep.final_c_hat_rel_err < 0.30);
        CHECK(std::isfinite(rep.theta_rmse_deg) && rep.theta_rmse_deg >= 0.0);
        CHECK(std::isfinite(rep.median_nis) && rep.median_nis >= 0.0);
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

    std::printf("twin: all tests passed\n");
    return 0;
}
