// Framework-free tests for the WP1 relative-motion module. Every check uses an
// explicit `return 1` on failure (R4: tests must fail in every build type, not
// only where assert() is live), so this behaves identically in Debug and
// Release. All inputs are fixed constants (R6: reproducible, no RNG).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "adsc/mission.hpp"
#include "adsc/relmotion.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

static double max_abs(const Vector6d& v) { return v.cwiseAbs().maxCoeff(); }

int main() {
    // Reference target orbit: SL-16 / Zenit-2 second-stage band (D2).
    const double a = kEarthRadius + 825.0e3;
    const CwModel cw = CwModel::from_orbit(a);

    // 1. Mean motion & period are the closed-form values (~101 min LEO period).
    {
        const double n_expected = std::sqrt(kEarthMu / (a * a * a));
        CHECK(std::abs(cw.n() - n_expected) < 1e-15);
        CHECK(std::abs(cw.period() - 2.0 * kPi / n_expected) < 1e-6);
        CHECK(cw.period() > 6000.0 && cw.period() < 6200.0);
    }

    // 2. Analytic STM agrees with independent RK4 integration (the spec asks the
    //    two propagators to be tested against each other).
    {
        std::vector<Vector6d> states;
        Vector6d s1;
        s1 << 100.0, -250.0, 40.0, 0.2, -0.1, 0.05;
        Vector6d s2;
        s2 << -500.0, 300.0, -120.0, -0.05, 0.3, -0.2;
        Vector6d s3;
        s3 << 0.0, 0.0, 0.0, 1.0, -1.0, 0.5;
        states.push_back(s1);
        states.push_back(s2);
        states.push_back(s3);

        const double T = cw.period();
        const std::vector<double> times = {0.10 * T, 0.37 * T, 0.50 * T,
                                           0.83 * T, 1.00 * T};
        for (const Vector6d& x0 : states) {
            for (double t : times) {
                const Vector6d xa = cw.propagate(x0, t);           // analytic
                const Vector6d xn = cw.propagate_rk4(x0, t, 0.05);  // RK4
                CHECK(max_abs(xa - xn) < 1e-3);
            }
        }
    }

    // 3. The safety ellipse is a genuine drift-free CW solution: vy0 = -2 n x0
    //    at every phase, and advancing the state by time dt equals advancing the
    //    ellipse phase by n*dt.
    {
        const SafetyEllipse e{400.0, 400.0, 0.0};
        const double n = cw.n();
        for (int i = 0; i < 8; ++i) {
            const double theta = 2.0 * kPi * static_cast<double>(i) / 8.0;
            const Vector6d x = cw.ellipse_state(e, theta);
            CHECK(std::abs(x(4) - (-2.0 * n * x(0))) < 1e-9);

            const double dt = 0.13 * cw.period();
            const Vector6d xa = cw.propagate(x, dt);
            const Vector6d xe = cw.ellipse_state(e, theta + n * dt);
            CHECK(max_abs(xa - xe) < 1e-6);
        }
    }

    // 4. Passive safety (D5 / WP1 acceptance): from >= 100 sampled points of the
    //    nominal approach, a thrust-off coast over >= 2 target orbital periods
    //    never enters the keep-out sphere.
    {
        const double keep_out = 200.0;  // [m]
        const std::vector<SafetyEllipse> corridor =
            approach_corridor(1200.0, 300.0, keep_out, 10);
        CHECK(!corridor.empty());

        const double horizon = 2.0 * cw.period();
        const double dt = 1.0;
        const int phases = 12;

        int samples = 0;
        double worst = std::numeric_limits<double>::max();
        for (const SafetyEllipse& hold : corridor) {
            CHECK(hold.min_range() > keep_out);  // safe by construction
            for (int p = 0; p < phases; ++p) {
                const double theta = 2.0 * kPi * static_cast<double>(p) / phases;
                Vector6d x = cw.ellipse_state(hold, theta);
                double local_min = rel_range(x);
                double t = 0.0;
                while (t < horizon) {
                    x = cw.propagate_rk4(x, dt, dt);
                    local_min = std::min(local_min, rel_range(x));
                    t += dt;
                }
                CHECK(local_min > keep_out);  // safe under numerical coast
                worst = std::min(worst, local_min);
                ++samples;
            }
        }
        CHECK(samples >= 100);
        std::printf(
            "relmotion: %d coast samples, worst closest-approach %.1f m "
            "> keep-out %.1f m\n",
            samples, worst, keep_out);
    }

    // 5. Safe-abort, uncapped (Clean): applying the CW abort impulse from a
    //    standoff with an inward closing rate yields a coast that stays outside
    //    keep-out, and the reported coast_min_range_m matches an independent
    //    propagation.
    {
        const Mission m;
        const Config cfg = m.config();
        const CwModel cwm =
            CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);

        const Eigen::Vector3d r_rel(40.0, -500.0, 15.0);
        const Eigen::Vector3d v_rel(-0.05, 0.02, -0.01);  // closing
        const SafeAbort ab = m.compute_safe_abort(r_rel, v_rel);
        CHECK(ab.status == SafeAbort::Status::Clean);

        Vector6d x;
        x << r_rel, (v_rel + ab.dv);

        const double horizon = cfg.abort_coast_check_periods * cwm.period();
        const double dt = cfg.abort_coast_check_dt_s;
        double local_min = rel_range(x);
        double t = 0.0;
        while (t < horizon) {
            x = cwm.propagate_rk4(x, dt, dt);
            local_min = std::min(local_min, rel_range(x));
            t += dt;
        }
        CHECK(local_min > cfg.keep_out_radius_m);
        CHECK(std::abs(local_min - ab.coast_min_range_m) < 1e-6);
    }

    // 6. Safe-abort, capped (F1): a geometry whose drift-nulling impulse
    //    exceeds the budget. The function must flag Capped, deliver exactly the
    //    capped magnitude, leave a genuine residual drift rate, and report a
    //    coast minimum range that an independent propagation reproduces —
    //    honesty instead of an implied bounded-orbit guarantee.
    {
        const Mission m;
        const Config cfg = m.config();
        const CwModel cwm =
            CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);

        // At x = 1200 m the drift-nulling delta-v is |-2 n x| ~ 2.48 m/s,
        // above the 2.0 m/s budget.
        const Eigen::Vector3d r_rel(1200.0, 0.0, 0.0);
        const Eigen::Vector3d v_rel(0.0, 0.0, 0.0);
        CHECK(2.0 * cwm.n() * r_rel.x() > cfg.abort_dv);  // premise of the case

        const SafeAbort ab = m.compute_safe_abort(r_rel, v_rel);
        CHECK(ab.status == SafeAbort::Status::Capped);
        CHECK(std::abs(ab.dv.norm() - cfg.abort_dv) < 1e-9);

        // Residual along-track rate w.r.t. the drift-free condition vy = -2nx:
        // the capped burn does NOT null the drift.
        const double vy_resid = ab.dv.y() + 2.0 * cwm.n() * r_rel.x();
        CHECK(std::abs(vy_resid) > 0.1);

        // The reported coast range is a real propagated number, not a promise.
        Vector6d x;
        x << r_rel, (v_rel + ab.dv);
        const double horizon = cfg.abort_coast_check_periods * cwm.period();
        const double dt = cfg.abort_coast_check_dt_s;
        double local_min = rel_range(x);
        double t = 0.0;
        while (t < horizon) {
            x = cwm.propagate_rk4(x, dt, dt);
            local_min = std::min(local_min, rel_range(x));
            t += dt;
        }
        CHECK(std::abs(local_min - ab.coast_min_range_m) < 1e-6);
    }

    std::printf("relmotion: all tests passed\n");
    return 0;
}
