// WP11 tests: bounded_coast_min_range cross-validation, the clearing-abort
// law's escalation ladder (Clean / BoundedClearing / RetreatHop), and the
// closed-loop translation-guidance demo (GuidedApproach). Framework-free
// (see tests/test_relmotion.cpp for the assertion-macro style); explicit
// return-1 checks (R4); all inputs fixed constants (R6: no RNG).
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

#include "adsc/guidance.hpp"
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

namespace {

// Independent cross-validation of bounded_coast_min_range: propagate the
// SAME drift-free state (vy0 = -2 n x0) via the fine RK4 sweep already used
// elsewhere (dt = 1 s, one orbital period) and compare minima.
double fine_rk4_min_range(const CwModel& cw, double x0, double y0, double z0,
                          double vx, double vz) {
    const double n = cw.n();
    Vector6d x;
    x << x0, y0, z0, vx, -2.0 * n * x0, vz;
    double min_range = rel_range(x);
    const double dt = 1.0;
    const double horizon = cw.period();
    double t = 0.0;
    while (t < horizon) {
        x = cw.propagate_rk4(x, dt, dt);
        min_range = std::min(min_range, rel_range(x));
        t += dt;
    }
    return min_range;
}

}  // namespace

int main() {
    // 1. bounded_coast_min_range cross-validation against an independent fine
    //    RK4 sweep, for a handful of states (including a forensic-14-like
    //    state with vx = vz = 0, per WP10c/generated/wp10_violation_forensics.csv).
    {
        const CwModel cw_825 = CwModel::from_orbit(kEarthRadius + 825.0e3);
        const CwModel cw_750 = CwModel::from_orbit(kEarthRadius + 750.0e3);

        struct Case { const CwModel* cw; double x0, y0, z0, vx, vz; };
        const Case cases[] = {
            {&cw_825, 100.0, -450.0, 30.0, 0.0, 0.0},
            {&cw_750, -136.6, -326.5, -67.5, 0.0, 0.0},  // forensic-14-like (SL-8, run 254)
            {&cw_825, 50.0, -350.0, -20.0, 0.02, -0.01},
        };
        for (const Case& c : cases) {
            const double analytic =
                bounded_coast_min_range(c.x0, c.y0, c.z0, c.vx, c.vz, c.cw->n());
            const double fine = fine_rk4_min_range(*c.cw, c.x0, c.y0, c.z0, c.vx, c.vz);
            CHECK(std::abs(analytic - fine) < 0.5);
        }
    }

    // 2. compute_clearing_abort escalation ladder.
    {
        // (a) forensic-14 state SL-8/254: legacy law's ellipse intersects
        //     keep-out; the WP11 law escalates to BoundedClearing and clears.
        Config cfg_a;
        cfg_a.target_altitude_km = 750.0;
        const Mission m_a(cfg_a);
        const CwModel cw_a = CwModel::from_orbit(kEarthRadius + 750.0e3);
        const Eigen::Vector3d r_a(-136.6, -326.5, -67.5);
        const Eigen::Vector3d v_a(-0.0146, 0.0038, -0.0026);
        CHECK(bounded_coast_min_range(r_a.x(), r_a.y(), r_a.z(), 0.0, 0.0, cw_a.n()) < 200.0);
        const SafeAbort ab_a = m_a.compute_clearing_abort(r_a, v_a);
        CHECK(ab_a.status == SafeAbort::Status::BoundedClearing);
        CHECK(ab_a.bounded_min_range_m >= 220.0);
        CHECK(ab_a.dv.norm() <= 2.0);

        // (b) inside-sphere state (0, -150, 0), v = 0: the drift-null
        //     ellipse and the radial reshape both fail (x0 = 0 leaves no
        //     radial DOF), so the law escalates to the two-impulse
        //     RetreatHop. Its transient leg opens monotonically (the
        //     minimum is the range at abort, ~150 m -- tight bound 149 m),
        //     and the terminal ellipse clears keep-out + margin (220 m).
        Config cfg_b;
        cfg_b.target_altitude_km = 750.0;
        const Mission m_b(cfg_b);
        const Eigen::Vector3d r_b(0.0, -150.0, 0.0);
        const Eigen::Vector3d v_b(0.0, 0.0, 0.0);
        const SafeAbort ab_b = m_b.compute_clearing_abort(r_b, v_b);
        CHECK(ab_b.status == SafeAbort::Status::RetreatHop);
        CHECK(ab_b.coast_min_range_m >= 149.0);
        CHECK(ab_b.bounded_min_range_m >= 220.0);
        CHECK(ab_b.dv.norm() + ab_b.dv_second_burn_m_s <= 2.0);

        // Escalation, not the default: the SAME forensic-14 state as (a) but
        // with v = 0 must still resolve at Stage 2 (BoundedClearing) -- the
        // RetreatHop hop is reached only when Stages 1 and 2 both fail.
        const Eigen::Vector3d v_zero(0.0, 0.0, 0.0);
        const SafeAbort ab_a0 = m_a.compute_clearing_abort(r_a, v_zero);
        CHECK(ab_a0.status == SafeAbort::Status::BoundedClearing);

        // (c) far outside, drift-null already clears comfortably: Clean.
        Config cfg_c;
        cfg_c.target_altitude_km = 750.0;
        const Mission m_c(cfg_c);
        const Eigen::Vector3d r_c(0.0, -400.0, 0.0);
        const Eigen::Vector3d v_c(0.0, 0.0, 0.0);
        const SafeAbort ab_c = m_c.compute_clearing_abort(r_c, v_c);
        CHECK(ab_c.status == SafeAbort::Status::Clean);
        CHECK(std::abs(ab_c.bounded_min_range_m - 400.0) < 1e-6);
    }

    // 3. guidance_mode_table(): non-empty, unique labels, every row filled in.
    {
        const std::vector<GuidanceModeSpec> table = guidance_mode_table();
        CHECK(!table.empty());
        for (std::size_t i = 0; i < table.size(); ++i) {
            const GuidanceModeSpec& row = table[i];
            CHECK(row.entry != nullptr && row.entry[0] != '\0');
            CHECK(row.exit != nullptr && row.exit[0] != '\0');
            CHECK(row.abort_condition != nullptr && row.abort_condition[0] != '\0');
            CHECK(row.abort_action != nullptr && row.abort_action[0] != '\0');
            for (std::size_t j = i + 1; j < table.size(); ++j) {
                CHECK(std::string(guidance_mode_label(row.mode)) !=
                      guidance_mode_label(table[j].mode));
            }
        }
    }

    // 4. GuidedApproach::fly(): the deterministic L0 demo completes, the
    //    reachability screen and LOS cone hold at every step, the achieved
    //    contact speed is produced by construction (not merely gated), and
    //    the total maneuver cost is sane.
    {
        const Config cfg;
        GuidedApproach approach(cfg);
        const GuidedApproachReport rep = approach.fly();

        CHECK(rep.completed);
        CHECK(rep.final_mode == GuidanceMode::Complete);
        CHECK(rep.contact_speed_m_s <= cfg.max_v_rel);
        CHECK(std::abs(rep.contact_speed_m_s - cfg.guid_contact_speed_m_s) < 1e-9);
        CHECK(rep.abort_feasible_every_step);
        CHECK(rep.los_cone_ok);
        CHECK(rep.min_clearance_outside_m > 0.0);
        CHECK(rep.dv_total_m_s > 0.0 && rep.dv_total_m_s < 140.0);
        CHECK(!rep.phases.empty());
    }

    std::printf("guidance: bounded_coast_min_range cross-validation, "
               "clearing-abort escalation ladder, and GuidedApproach demo OK\n");
    return 0;
}
