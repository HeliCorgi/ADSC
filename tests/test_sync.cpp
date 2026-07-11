// WP2 tests: tracking-SMC feedforward honesty, tumble-sync acceptance, the
// regulation/tracking equivalence, and the v2 detumble regression pin.
// Explicit return-1 checks (R4), fixed constants only, no randomness (R6).
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "adsc/controller.hpp"
#include "adsc/dynamics.hpp"
#include "adsc/mission.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

int main() {
    const double deg2rad = kPi / 180.0;

    // 1. Feedforward honesty (the anti-self-deception test the spec's WP2
    //    derivation notes call for). The SMC reaching term is robust enough to
    //    mop up a wrong feedforward and still pass the acceptance criteria, so
    //    the acceptance test cannot certify the feedforward. Here the reaching
    //    term is disabled outright (k = 0, deadband = 0): the servicer flies on
    //    equivalent control + feedforward alone, with nothing to rescue it.
    //    Starting perfectly synchronized with a fast, strongly precessing
    //    tumble (5 deg/s, the top of the spec range), a CORRECT feedforward
    //    parks the sliding variable at integration-error level (~1e-4 expected
    //    from the zero-order-hold torque at dt = 0.01); dropping the w_t_dot
    //    feedforward alone integrates |s| to ~0.11 over 30 s (>20x the limit
    //    below), so this test fails loudly on a wrong w_t_dot feedforward.
    //    Honest scope note: on a matched trajectory w_e ~ 0, so a wrong
    //    TRANSPORT term (-w_e x C^T w_t) is near-invisible here; that term is
    //    exercised by the acceptance test's convergence from 40 deg instead.
    {
        const Config cfg;
        SlidingModeController::Gains g;
        g.k          = 0.0;   // reaching OFF: nothing can hide a feedforward error
        g.deadband   = 0.0;   // continuous torque (a coasting deadband would
                              // itself force drift; see the sync_gains comment)
        g.max_torque = 1.0;   // far above the ~6e-3 N m feedforward level
        const SlidingModeController ctrl(g);

        RigidBody target(Eigen::Vector3d(1.0, 0.6, 0.3).asDiagonal(),
                         Eigen::Quaterniond::Identity(),
                         5.0 * deg2rad *
                             Eigen::Vector3d(0.5, 0.7, -0.5).normalized());
        // Perfectly matched start: same attitude, same body rate.
        RigidBody servicer(Eigen::Matrix3d::Identity() * cfg.base_inertia,
                           Eigen::Quaterniond::Identity(), target.rate());

        const Eigen::Matrix3d I_t     = target.inertia();
        const Eigen::Matrix3d I_t_inv = I_t.inverse();

        double max_s = 0.0;
        double t = 0.0;
        while (t < 30.0) {
            const Eigen::Vector3d w_t = target.rate();
            const Eigen::Vector3d w_t_dot = I_t_inv * (-(w_t.cross(I_t * w_t)));
            const Eigen::Vector3d tau = ctrl.torque(
                servicer.inertia(), servicer.attitude(), servicer.rate(),
                target.attitude(), w_t, w_t_dot);
            servicer.step(tau, cfg.control_dt);
            target.step(Eigen::Vector3d::Zero(), cfg.control_dt);
            t += cfg.control_dt;

            const Eigen::Vector3d s = ctrl.sliding_surface(
                servicer.attitude(), servicer.rate(),
                target.attitude(), target.rate());
            max_s = std::max(max_s, s.cwiseAbs().maxCoeff());
        }
        std::printf("sync: feedforward-honesty max|s|_inf = %.3e over 30 s "
                    "(limit 5e-3; default deadband 1.5e-2)\n", max_s);
        CHECK(max_s < 5.0e-3);
    }

    // 2. Acceptance (spec section 5 WP2): |w_rel| < 0.1 deg/s and attitude
    //    error < 2 deg held for 30 s against a 2 deg/s precessing tumble,
    //    starting from a 40 deg attitude offset at zero rate. Same fixed
    //    scenario as the adsc_sim demo (R6). The report's max errors run from
    //    the end of the dwell window to the END of the 120 s run, so this
    //    asserts that sync does not degrade after it is declared.
    {
        const Config cfg;
        const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d w_t0 =
            cfg.sync_target_rate_deg_s * deg2rad *
            Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
        const Eigen::Quaterniond q_c0(
            Eigen::AngleAxisd(40.0 * deg2rad,
                              Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

        const SyncReport rep = run_tumble_sync(
            cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), 120.0);

        std::printf("sync: acceptance sync_time = %.2f s; since sync "
                    "max|w_rel| = %.4f deg/s, max att err = %.4f deg\n",
                    rep.sync_time_s, rep.max_rate_err_deg_s,
                    rep.max_att_err_deg);
        CHECK(rep.synced);
        CHECK(rep.sync_time_s < 90.0);
        CHECK(rep.max_rate_err_deg_s < cfg.sync_rate_tol_deg_s);
        CHECK(rep.max_att_err_deg < cfg.sync_att_tol_deg);
    }

    // 3. Regulation == tracking with w_t = 0 (the spec's "keep regulation as
    //    the special case"). The regulation overload is a separate verbatim
    //    code path for floating-point regression safety, so the special-case
    //    property is asserted here numerically on a spread of states.
    {
        const SlidingModeController ctrl;  // default gains
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity() * 0.55;
        const Eigen::Quaterniond targets[2] = {
            Eigen::Quaterniond::Identity(),
            Eigen::Quaterniond(Eigen::AngleAxisd(
                25.0 * deg2rad, Eigen::Vector3d(0.2, -1.0, 0.4).normalized())),
        };
        const Eigen::Quaterniond qs[3] = {
            Eigen::Quaterniond(Eigen::AngleAxisd(
                40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized())),
            Eigen::Quaterniond(Eigen::AngleAxisd(
                160.0 * deg2rad, Eigen::Vector3d(-0.3, 0.8, 0.5).normalized())),
            Eigen::Quaterniond::Identity(),
        };
        const Eigen::Vector3d ws[3] = {
            {0.15, -0.12, 0.09}, {-0.02, 0.005, 0.001}, {0.0, 0.0, 0.0}};

        for (const auto& q_t : targets) {
            for (const auto& q : qs) {
                for (const auto& w : ws) {
                    const Eigen::Vector3d tau_reg = ctrl.torque(I, q, w, q_t);
                    const Eigen::Vector3d tau_trk = ctrl.torque(
                        I, q, w, q_t, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero());
                    CHECK((tau_reg - tau_trk).cwiseAbs().maxCoeff() < 1.0e-10);
                }
            }
        }
    }

    // 4. v2 detumble regression pin (R1 guard): the quoted reference numbers
    //    (settle 19.15 s, final rate 3.61e-4 rad/s) must keep reproducing.
    {
        Mission mission;
        auto rep = mission.post_capture_stabilization(
            true, 2.4, Eigen::Vector3d(0, 0, 0.15),
            Eigen::Vector3d(0.08, 0.04, 0.11),
            Eigen::Vector3d(0.12, 0.05, 0.04),
            Eigen::Vector3d(0.15, -0.12, 0.09));
        std::printf("sync: v2 regression settle = %.4f s, final rate = %.6f rad/s\n",
                    rep.settle_time_s, rep.final_rate);
        CHECK(rep.captured);
        CHECK(rep.settled);
        CHECK(std::abs(rep.settle_time_s - 19.15) < 0.02);
        CHECK(rep.final_rate < 5.0e-4);
    }

    std::printf("sync: all tests passed\n");
    return 0;
}
