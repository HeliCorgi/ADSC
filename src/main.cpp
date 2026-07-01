#include <cstdio>

#include "adsc/mission.hpp"

int main() {
    using namespace adsc;

    std::printf("=== ADSC v2.0 — Active Debris Self-Cleanup (redesign) ===\n\n");

    Mission mission;
    std::printf("Dry mass        : %.2f kg\n", mission.config().dry_mass_kg);
    std::printf("Initial fuel    : %.2f kg\n", mission.fuel_kg());
    std::printf("Control step dt : %.3f s\n\n", mission.config().control_dt);

    // --- Scenario 1: closing too fast -> safe abort ------------------------
    {
        Eigen::Vector3d r_rel(0.08, 0.04, 0.11);
        Eigen::Vector3d v_fast(0.20, 0.10, 0.10);   // |v| ~ 0.245 m/s > cap
        auto rep = mission.post_capture_stabilization(
            true, 2.4, Eigen::Vector3d(0, 0, 0.15), r_rel, v_fast,
            Eigen::Vector3d(0.05, -0.04, 0.03));
        std::printf("[Scenario 1] closing speed %.3f m/s -> %s\n\n",
                    v_fast.norm(), rep.aborted ? "ABORT (safe maneuver)" : "captured");
    }

    // --- Scenario 2: valid capture -> closed-loop detumble -----------------
    {
        Eigen::Vector3d r_rel(0.08, 0.04, 0.11);
        Eigen::Vector3d v_ok(0.12, 0.05, 0.04);      // |v| ~ 0.14 m/s < cap
        Eigen::Vector3d tumble(0.15, -0.12, 0.09);   // post-capture body rate
        auto rep = mission.post_capture_stabilization(
            true, 2.4, Eigen::Vector3d(0, 0, 0.15), r_rel, v_ok, tumble);

        std::printf("[Scenario 2] captured=%s\n", rep.captured ? "yes" : "no");
        std::printf("  mass after capture : %.2f kg\n", rep.mass_total_kg);
        std::printf("  inertia trace      : %.4f kg m^2\n", rep.inertia_trace);
        std::printf("  initial rate |w|   : %.4f rad/s\n", tumble.norm());
        std::printf("  final   rate |w|   : %.6f rad/s\n", rep.final_rate);
        std::printf("  settled            : %s\n", rep.settled ? "yes" : "no");
        if (rep.settled)
            std::printf("  settle time        : %.2f s\n", rep.settle_time_s);
        std::printf("  thermal ok         : %s\n\n", rep.thermal_ok ? "yes" : "SAFE MODE");
    }

    // --- Deorbit gating ----------------------------------------------------
    {
        bool autonomous = false;
        bool ok = mission.deorbit_permitted(/*ground_human_approval=*/false, autonomous);
        std::printf("[Deorbit] permitted=%s mode=%s\n",
                    ok ? "yes" : "no", autonomous ? "autonomous" : "human-in-the-loop");
    }

    std::printf("\n=== simulation complete ===\n");
    return 0;
}
