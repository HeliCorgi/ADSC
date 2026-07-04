// WP11 forensic-14 regression (D13 / R15). forensic-14: the keep-out-violating
// runs of the ds-v1 campaign under the LEGACY abort law, classified in
// generated/wp10_violation_forensics.csv (WP10c, PR #17); pinned per WP11 as a
// permanent regression: the legacy law reproduces the violation mechanism, the
// WP11 clearing law clears every case. Explicit return-1 checks (R4); every
// input below is a fixed, committed-generated-value pin (R15), same status as
// the 19.15/424.3/16.87/17.07 s reference numbers.
#include <cstdio>

#include "adsc/campaign.hpp"
#include "adsc/decay.hpp"
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

// One pinned violating-abort state. `catalog` is 'A' (SL-16 / Zenit-2, 840 km,
// catalog_A()) or 'B' (SL-8 / Kosmos-3M, 750 km, catalog_B()). (x0,y0,z0,
// vx0,vy0,vz0) are copied verbatim from generated/wp10_violation_forensics.csv
// columns r0_x_m, r0_y_m, r0_z_m, v0_x_m_per_s, v0_y_m_per_s, v0_z_m_per_s
// (WP10c committed-generated values; regression pins per R15, same status as
// 19.15 s etc. -- do not hand-edit if the forensics ever regenerate).
struct ForensicCase {
    char   catalog;
    int    run_index;
    double x0, y0, z0, vx0, vy0, vz0;
};

const ForensicCase kForensic14[] = {
    {'A', 65,  -109.2047, -390.9569,  47.6195,  0.053575,  0.002560, -0.004322},
    {'A', 81,   -62.6103, -295.8113,   3.2022, -0.015877, -0.010833,  0.008057},
    {'A', 86,    88.9285, -377.7154, -36.4953, -0.033824,  0.021650, -0.010427},
    {'A', 112,  103.1442, -403.3742, -17.1617, -0.005783, -0.001262, -0.006628},
    {'A', 340,  142.3706, -444.1451,  39.5522,  0.014951, -0.009629,  0.020602},
    {'A', 460,  -80.9837, -340.0491, -30.4610, -0.019190, -0.032109,  0.028635},
    {'A', 490, -136.5058, -403.0482,  31.5876,  0.000136, -0.023902, -0.020074},
    {'B', 88,  -110.7634, -340.9315, -23.5717,  0.024717,  0.019986, -0.001310},
    {'B', 99,   -78.3053, -281.4154,  -6.6638, -0.015060, -0.001444, -0.005980},
    {'B', 160, -126.4040, -380.8614,  20.7493, -0.005108,  0.007905,  0.005312},
    {'B', 254, -136.6149, -326.4996, -67.5308, -0.014607,  0.003844, -0.002598},
    {'B', 362,  -88.5700, -376.2516, 125.0294,  0.021529,  0.000352,  0.033527},
    {'B', 383,  -81.6652, -346.3719,  39.5522, -0.010185,  0.011825,  0.016236},
    {'B', 419,  107.7674, -403.6285,  85.6915, -0.013524,  0.015787, -0.004265},
};
const int kNumForensic = static_cast<int>(
    sizeof(kForensic14) / sizeof(kForensic14[0]));

const int kRunsA[] = {65, 81, 86, 112, 340, 460, 490};
const int kRunsB[] = {88, 99, 160, 254, 362, 383, 419};

}  // namespace

int main() {
    // Part 1: mechanism reproduction (legacy law) + WP11 fix, evaluated
    // directly from the pinned committed states (no campaign internals
    // duplicated).
    for (int i = 0; i < kNumForensic; ++i) {
        const ForensicCase& c = kForensic14[i];
        const double alt_km = (c.catalog == 'A') ? 840.0 : 750.0;

        Config cfg;
        cfg.target_altitude_km = alt_km;
        const Mission m(cfg);
        const CwModel cw = CwModel::from_orbit(kEarthRadius + alt_km * 1000.0);
        const double n = cw.n();

        // (i) Legacy mechanism reproduced analytically: the drift-null
        //     safety ellipse (compute_safe_abort's target orbit) dips below
        //     the keep-out radius.
        const double legacy_ellipse_min =
            bounded_coast_min_range(c.x0, c.y0, c.z0, 0.0, 0.0, n);
        CHECK(legacy_ellipse_min < 200.0);

        // (ii) WP11 clearing law clears every case.
        const Eigen::Vector3d r(c.x0, c.y0, c.z0);
        const Eigen::Vector3d v(c.vx0, c.vy0, c.vz0);
        const SafeAbort ab = m.compute_clearing_abort(r, v);
        CHECK(ab.status == SafeAbort::Status::BoundedClearing ||
              ab.status == SafeAbort::Status::Clean);
        CHECK(ab.bounded_min_range_m > 200.0);
        CHECK(ab.coast_min_range_m > 200.0);
        CHECK(ab.dv.norm() <= 2.0);
    }
    std::printf(
        "forensic14: all %d pinned states -- legacy mechanism reproduced "
        "analytically, WP11 clearing law clears every case\n",
        kNumForensic);

    // Part 2: the fix holds in the real campaign path (run_one_mission),
    // under the standard CampaignConfig + the main_campaign coarsened
    // keep-out-screen overrides (1 orbital period at an 8 s RK4 step).
    const CampaignConfig ccfg;
    Config base_cfg;
    base_cfg.abort_coast_check_periods = 1.0;
    base_cfg.abort_coast_check_dt_s    = 8.0;
    const DebrisCatalog A = catalog_A();
    const DebrisCatalog B = catalog_B();

    for (int idx : kRunsA) {
        const RunResult r = run_one_mission(A, ccfg, base_cfg, idx);
        CHECK(r.outcome != Outcome::KeepOutViolation);
        CHECK(!r.keep_out_violation);
        CHECK(r.gate_abort_events > 0);
        CHECK(r.worst_abort_clearance_m >= 0.0);
    }
    for (int idx : kRunsB) {
        const RunResult r = run_one_mission(B, ccfg, base_cfg, idx);
        CHECK(r.outcome != Outcome::KeepOutViolation);
        CHECK(!r.keep_out_violation);
        CHECK(r.gate_abort_events > 0);
        CHECK(r.worst_abort_clearance_m >= 0.0);
    }
    std::printf(
        "forensic14: all 14 pinned (catalog, run_index) pairs replay clean "
        "under run_one_mission with the WP11 clearing law\n");
    return 0;
}
