// WP3 mission-flow tests: the installer phase sequence
// (approach -> sync -> attach -> depart), the attach mass/area handover, and
// the contact-honesty energy budget. Explicit return-1 checks (R4), fixed
// constants, no RNG (R6).
#include <cmath>
#include <cstdio>

#include "adsc/decay.hpp"     // catalog_A for the target mass
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

    Mission mission;
    const Config cfg = mission.config();

    // Same tumble/offset as the WP2 acceptance scenario (known to synchronize).
    const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                 Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
    const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
        40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

    const DebrisCatalog target = catalog_A();

    // 1. Full installer mission reaches every phase.
    const MissionReport rep =
        mission.run_installer_mission(target.mass_kg, q_t0, w_t0, q_c0, 120.0);
    CHECK(rep.approach_safe);
    CHECK(rep.synced);
    CHECK(rep.attach.clamped);
    CHECK(rep.departed);
    CHECK(rep.success);
    CHECK(rep.reached == Phase::Complete);

    // 2. Contact-honesty budget (spec v4.1 WP3): 0.5 * (dry+kit) * max_v_rel^2,
    //    ~0.33 J for the reference 29.6 kg at 0.15 m/s.
    const double expect_ce = 0.5 * (cfg.dry_mass_kg + cfg.kit_mass_kg) *
                             cfg.max_v_rel * cfg.max_v_rel;
    std::printf("mission: contact mass %.1f kg, contact energy %.4f J\n",
                cfg.dry_mass_kg + cfg.kit_mass_kg, rep.attach.contact_energy_j);
    CHECK(std::abs(rep.attach.contact_energy_j - expect_ce) < 1e-12);
    CHECK(std::abs(rep.attach.contact_energy_j - 0.333) < 1e-3);

    // 3. Kit handover: servicer loses the kit, target gains kit mass + sail area.
    CHECK(std::abs(rep.attach.servicer_mass_before_kg -
                   (cfg.dry_mass_kg + cfg.kit_mass_kg)) < 1e-12);
    CHECK(std::abs(rep.attach.servicer_mass_after_kg - cfg.dry_mass_kg) < 1e-12);
    CHECK(std::abs(rep.attach.target_mass_after_kg -
                   (target.mass_kg + cfg.kit_mass_kg)) < 1e-9);
    CHECK(std::abs(rep.attach.target_area_after_m2 - cfg.kit_sail_area_m2) < 1e-12);
    const double expect_am = cfg.kit_sail_area_m2 / (target.mass_kg + cfg.kit_mass_kg);
    CHECK(std::abs(rep.attach.target_area_over_mass - expect_am) < 1e-15);

    // 4. Departure onto a bounded relative orbit clears the keep-out.
    CHECK(rep.depart_coast_min_m > cfg.keep_out_radius_m);

    // 5. Sync gate: an impossibly short sync budget aborts before attach.
    const MissionReport rep2 =
        mission.run_installer_mission(target.mass_kg, q_t0, w_t0, q_c0, 1.0);
    CHECK(!rep2.synced);
    CHECK(!rep2.attach.clamped);
    CHECK(!rep2.success);
    CHECK(rep2.reached == Phase::Aborted);

    std::printf("mission: installer flow approach->sync->attach->depart OK\n");
    return 0;
}
