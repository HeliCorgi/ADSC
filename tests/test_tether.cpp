// WP16 tether-model tests (Deliverables 1-5): energy-audit sanity, the
// tension-only slack invariant, the dumbbell-limit validation (T4a, loose
// tolerance -- see the note at test 3), divergence-guard triggering,
// controller pure functions, determinism, and the libration_eps helper
// against the T7 nominal value. Explicit return-1 checks (R4); this file
// carries no RNG at all (R6) -- tether.cpp is a pure function of its
// TetherConfig.
//
// [DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]. T7 stays OPEN;
// nothing here claims otherwise.
//
// [DT-v2: 3D] NEW blocks (8-12) below cover the WP16 Phase-2 out-of-plane
// extension (existing blocks 1-7 above are byte-untouched): the 3D energy
// audit, the z-mode planar-decoupling bit-identity guarantee, the D9
// dumbbell-limit-vs-T7-2DOF acceptance bands, the cone-angle 3D divergence
// guard trip, and the free-libration frequency checks (omega_pitch~sqrt(3)n,
// omega_roll~2n) that back up the D9 model-family-offset claim
// (main_twin.cpp's run_dumbbell_validation_3d comment). T7 stays OPEN here
// too; nothing in these blocks claims otherwise.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "adsc/tether.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

namespace {

// [DT-v2: 3D] free-libration period measurement: steps sim until the first
// two local maxima of angle_fn(sim) are found, returns the time between
// them [s] (-1.0 if fewer than two peaks turn up within max_steps).
double measure_period_s(TetherSim& sim, double (*angle_fn)(const TetherSim&), int max_steps) {
    double prev2 = angle_fn(sim);
    double prev1 = prev2;
    std::vector<double> peak_times;
    for (int i = 0; i < max_steps && peak_times.size() < 2u; ++i) {
        sim.step();
        const double cur = angle_fn(sim);
        if (i >= 1 && prev1 > prev2 && prev1 > cur) {
            peak_times.push_back(sim.time_s() - sim.config().dt_s);  // time of the prev1 sample (the peak)
        }
        prev2 = prev1;
        prev1 = cur;
    }
    if (peak_times.size() < 2u) return -1.0;
    return peak_times[1] - peak_times[0];
}

double get_chord_angle(const TetherSim& s) { return s.chord_angle_rad(); }
double get_roll_angle(const TetherSim& s) { return s.roll_angle_rad(); }

}  // namespace

int main() {
    // 1. Energy audit: uncontrolled (I=0), no damping, small-angle, over 10
    //    orbits should show a small energy drift (Deliverable 3). The bound
    //    below is deliberately LOOSE: this Phase-1 implementation could not
    //    be empirically tuned against a local build (no C++ toolchain
    //    available in the authoring environment; verified via CI instead),
    //    so a generous bound is used to catch gross regressions (NaN,
    //    blowup, a sign error in the energy bookkeeping) rather than to pin
    //    an exact numeric drift figure.
    {
        TetherConfig cfg;
        cfg.n_beads = 8;
        cfg.theta0_deg = 1.0;
        cfg.const_current_a = 0.0;
        cfg.controller = ControllerMode::Constant;
        cfg.damping_c_Ns_per_m = 0.0;
        cfg.sim_orbits = 10.0;
        const SimResult r = run_tether_sim(cfg);
        CHECK(r.status == DivergeStatus::Ok);
        CHECK(std::isfinite(r.energy_drift_per_orbit));
        CHECK(r.energy_drift_per_orbit >= 0.0);
        CHECK(r.energy_drift_per_orbit < 0.2);
    }

    // 2. Slack segments never push: root_tension_n() (the SAME value the
    //    dynamics themselves apply, via the max(0,.) gate in
    //    tether.cpp's compute_forces) must never be negative. The scenario
    //    (soft spring, no damping, 45-deg off-equilibrium IC) is chosen to
    //    PLAUSIBLY exercise at least one slack segment (large-amplitude,
    //    undamped overshoot), but whether it actually does is not
    //    hard-asserted -- only printed informationally -- since that outcome
    //    could not be empirically pre-verified without a local build.
    {
        TetherConfig cfg;
        cfg.n_beads = 2;
        cfg.tether_length_m = 3000.0;
        cfg.m_parent_kg = 9000.0;
        cfg.m_tip_kg = 20.0;
        cfg.lambda_tether_kg_per_m = 0.0;
        cfg.EA_design_N = 1.0;         // deliberately very soft: encourages overshoot/slack
        cfg.damping_c_Ns_per_m = 0.0;  // no damping to suppress the overshoot
        cfg.altitude_km = 840.0;
        cfg.inclination_deg = 71.0;
        cfg.theta0_deg = 45.0;         // far off the theta=0 (radial) equilibrium
        cfg.controller = ControllerMode::Constant;
        cfg.const_current_a = 0.0;
        cfg.dt_s = 5.0;                // omega_s here is small (soft spring); dt sized accordingly

        TetherSim sim(cfg);
        bool saw_slack = false;
        const int n_test_steps = 2000;
        for (int i = 0; i < n_test_steps; ++i) {
            const StepDiagnostics d = sim.step();
            CHECK(d.root_tension_n >= -1e-9);
            if (d.n_slack > 0) saw_slack = true;
        }
        if (!saw_slack) {
            std::printf("tether: NOTE test 2 scenario did not exercise a slack "
                       "segment (informational only, not a failure)\n");
        }
    }

    // 3. Dumbbell-limit vs T7's reference numbers, LOOSE tolerance. Cites
    //    ONLY t7-libration-study.md Sec 5.1 (the 1-DOF table) -- the bands
    //    below are RK4 fixed-step vs T7's own adaptive DOP853 integrator,
    //    plus this being a from-scratch lumped-mass reimplementation with no
    //    local numeric pre-verification, so only ORDER-OF-MAGNITUDE
    //    agreement is asserted -- never a precise growth-rate match. Sec
    //    5.1's own "ADVERSARIAL CORRECTION" appendix (a 2-DOF, roll-coupled
    //    result, o45~0.53 orbit at this eps) is explicitly OUT OF SCOPE for
    //    this planar Phase-1 model (tether.hpp's own file-header scope
    //    statement: only the DC/in-plane B_n channel is integrated here) and
    //    is not what these bands check against.
    {
        TetherConfig base;
        base.n_beads = 2;
        base.tether_length_m = 3000.0;
        base.m_parent_kg = 9000.0;
        base.m_tip_kg = 20.0;
        base.lambda_tether_kg_per_m = 0.0;
        base.EA_design_N = 10000.0;     // rigid limit (omega_s/omega_pitch ~= 229 >= 200 guardrail)
        base.damping_c_Ns_per_m = 0.0;  // T7's own model carries no dashpot
        base.altitude_km = 840.0;
        base.inclination_deg = 71.0;
        base.theta0_deg = 3.0;          // T7 seed
        base.controller = ControllerMode::Constant;
        base.dt_s = 0.2;
        base.sim_orbits = 5.0;

        // eps=0.106: T7 says BOUNDED libration, max|theta|=9.26 deg, never tumbles.
        {
            TetherConfig cfg = base;
            cfg.const_current_a = current_for_eps(0.106, cfg.altitude_km, cfg.inclination_deg,
                                                  cfg.m_parent_kg, cfg.m_tip_kg,
                                                  cfg.tether_length_m);
            const SimResult r = run_tether_sim(cfg);
            CHECK(r.status == DivergeStatus::Ok);
            CHECK(r.max_chord_angle_deg > 3.0 && r.max_chord_angle_deg < 20.0);  // LOOSE band around T7's 9.26 deg
        }
        // eps=0.80: T7 says TUMBLE, o45 in [0.10,0.20] orbit (T7=0.14).
        {
            TetherConfig cfg = base;
            cfg.const_current_a = current_for_eps(0.80, cfg.altitude_km, cfg.inclination_deg,
                                                  cfg.m_parent_kg, cfg.m_tip_kg,
                                                  cfg.tether_length_m);
            const SimResult r = run_tether_sim(cfg);
            CHECK(std::isfinite(r.o45_orbit));
            CHECK(r.o45_orbit > 0.02 && r.o45_orbit < 0.5);  // LOOSE band around T7's 0.14 orbit
        }
    }

    // 4. Divergence guard fires on a forced blowup (eps=3.0, far past the
    //    eps=1/2 tumble threshold).
    {
        TetherConfig cfg;
        cfg.n_beads = 2;
        cfg.tether_length_m = 3000.0;
        cfg.m_parent_kg = 9000.0;
        cfg.m_tip_kg = 20.0;
        cfg.lambda_tether_kg_per_m = 0.0;
        cfg.EA_design_N = 10000.0;
        cfg.damping_c_Ns_per_m = 0.0;
        cfg.altitude_km = 840.0;
        cfg.inclination_deg = 71.0;
        cfg.theta0_deg = 3.0;
        cfg.controller = ControllerMode::Constant;
        cfg.const_current_a = current_for_eps(3.0, cfg.altitude_km, cfg.inclination_deg,
                                              cfg.m_parent_kg, cfg.m_tip_kg, cfg.tether_length_m);
        cfg.dt_s = 0.2;
        cfg.sim_orbits = 5.0;
        const SimResult r = run_tether_sim(cfg);
        CHECK(r.status != DivergeStatus::Ok);
        CHECK(r.divergence_orbit < 5.0);
        CHECK(std::string(diverge_status_label(r.status)) != "ok");
    }

    // 5. Controller pure functions (Deliverable 5) -- no simulation needed.
    {
        CHECK(controller_c1_gate(-1.0, 0.5, false) == true);
        CHECK(controller_c1_gate(1.0, 0.5, true) == false);
        CHECK(controller_c1_gate(0.0, 0.5, true) == true);    // hysteresis band: hold ON
        CHECK(controller_c1_gate(0.0, 0.5, false) == false);  // hysteresis band: hold OFF

        CHECK(controller_c2_gate(0.0, 0.75, 0.0) == true);
        CHECK(controller_c2_gate(2.0 * kPi * 0.5, 0.75, 0.0) == true);   // phase 0.5 < 0.75
        CHECK(controller_c2_gate(2.0 * kPi * 0.8, 0.75, 0.0) == false);  // phase 0.8 >= 0.75
        CHECK(controller_c2_gate(2.0 * kPi * 0.9, 0.75, 0.5) == true);   // phase (0.9+0.5) mod 1 = 0.4 < 0.75
    }

    // 6. Determinism: two runs of the same cfg are bit-identical (R6). This
    //    file has no RNG at all, so this also guards against uninitialized
    //    reads / iteration-order nondeterminism.
    {
        TetherConfig cfg;
        cfg.controller = ControllerMode::PhaseGated;
        cfg.sim_orbits = 2.0;
        const SimResult r1 = run_tether_sim(cfg);
        const SimResult r2 = run_tether_sim(cfg);
        CHECK(r1.status == r2.status);
        CHECK(r1.n_steps == r2.n_steps);
        CHECK(r1.max_chord_angle_deg == r2.max_chord_angle_deg);
        CHECK(r1.energy_drift_per_orbit == r2.energy_drift_per_orbit);
        CHECK(r1.eta_lib_effective == r2.eta_lib_effective);
    }

    // 7. libration_eps matches the T7 nominal value (I=2A, i=71deg,
    //    m_tip=20kg -> eps=0.106, t7-libration-study.md Sec 5.0), and
    //    current_for_eps is its inverse.
    {
        const double eps = libration_eps(2.0, 840.0, 71.0, 9000.0, 20.0, 3000.0);
        CHECK(std::abs(eps - 0.106) < 0.002);

        const double i_back = current_for_eps(eps, 840.0, 71.0, 9000.0, 20.0, 3000.0);
        CHECK(std::abs(i_back - 2.0) < 1e-9);
    }

    // 8. [DT-v2: 3D] z-mode planar-decoupling BIT-IDENTITY guarantee: with
    //    out_of_plane == false (the default every existing test/config
    //    above relies on), a NONZERO phi0_deg must still be completely
    //    IGNORED -- every bead's z position and velocity must stay at
    //    EXACTLY 0.0 (not merely "small") for the whole run, because
    //    tether.cpp's z-force terms are only ever ADDED under an
    //    out_of_plane branch, never computed unconditionally. This is the
    //    R1 no-fork / planar-bit-identity binding guarantee, tested
    //    directly (not just argued in a comment).
    {
        TetherConfig cfg;
        cfg.n_beads = 8;
        cfg.theta0_deg = 3.0;
        cfg.phi0_deg = 25.0;       // deliberately large and nonzero
        cfg.out_of_plane = false;  // MUST be ignored
        cfg.controller = ControllerMode::PhaseGated;  // exercises the field-dependent C1 gate path too
        cfg.sim_orbits = 2.0;

        TetherSim sim(cfg);
        for (int i = 0; i < cfg.n_beads; ++i) {
            CHECK(sim.state().pos[static_cast<std::size_t>(i)].z() == 0.0);
            CHECK(sim.state().vel[static_cast<std::size_t>(i)].z() == 0.0);
        }
        const double t_orbit = 2.0 * kPi / sim.mean_motion_rad_s();
        while (sim.time_s() < cfg.sim_orbits * t_orbit) {
            sim.step();
            for (int i = 0; i < cfg.n_beads; ++i) {
                CHECK(sim.state().pos[static_cast<std::size_t>(i)].z() == 0.0);
                CHECK(sim.state().vel[static_cast<std::size_t>(i)].z() == 0.0);
            }
            if (sim.status() != DivergeStatus::Ok) break;
        }
    }

    // 9. [DT-v2: 3D] 3D energy audit sanity: out_of_plane == true,
    //    uncontrolled (I=0), no damping, small angles, over 10 orbits --
    //    same loose-bound spirit as CHECK 1 (catch gross regressions: NaN,
    //    blowup, a sign error in the new z-terms), now with a genuinely
    //    nonzero roll IC so the z-dependent U_gg/z-accel code paths are
    //    actually exercised.
    {
        TetherConfig cfg;
        cfg.n_beads = 8;
        cfg.theta0_deg = 1.0;
        cfg.phi0_deg = 1.0;
        cfg.out_of_plane = true;
        cfg.const_current_a = 0.0;
        cfg.controller = ControllerMode::Constant;
        cfg.damping_c_Ns_per_m = 0.0;
        cfg.sim_orbits = 10.0;
        const SimResult r = run_tether_sim(cfg);
        CHECK(r.status == DivergeStatus::Ok);
        CHECK(std::isfinite(r.energy_drift_per_orbit));
        CHECK(r.energy_drift_per_orbit >= 0.0);
        CHECK(r.energy_drift_per_orbit < 0.2);
        CHECK(std::isfinite(r.max_cone_angle_deg));
        CHECK(r.max_cone_angle_deg >= 0.0);
        CHECK(r.max_roll_deg >= 0.0);
    }

    // 10. [DT-v2: 3D] D9 dumbbell-limit-vs-T7-2DOF acceptance anchor: N=2
    //     rigid massless dumbbell, out_of_plane=true, nominal I=2A/i=71deg
    //     (eps=0.106), theta0=phi0=3deg (T7's own seed). Target: T7's own
    //     ADVERSARIAL CORRECTION 2-DOF reference (o45~0.53 orbit, 80-deg
    //     divergence by ~4.35 orbits). Bands are LOOSE and pre-registered,
    //     set with margin around an independently-verified (from-scratch
    //     Python reimplementation, cross-checked convergent across EA and
    //     dt) bead-model result of o45(cone)~1.41 orbit, divergence~10.4
    //     orbits -- see main_twin.cpp's run_dumbbell_validation_3d comment
    //     for the full model-family-offset accounting (this bead model's
    //     free CoM absorbs a real net Lorentz force that T7's own
    //     attitude-only reduced Lagrangian excludes by construction). The
    //     LOAD-BEARING claim is QUALITATIVE: the 3D mode DIVERGES within
    //     O(10) orbits, sharply UNLIKE the planar mode's bounded ~15 deg at
    //     this identical nominal config (CHECK 3 above) -- precise
    //     growth-rate matching is explicitly NOT asserted (T7's own
    //     IC-fragility note: "precise rates are not robust").
    {
        TetherConfig cfg;
        cfg.n_beads = 2;
        cfg.tether_length_m = 3000.0;
        cfg.m_parent_kg = 9000.0;
        cfg.m_tip_kg = 20.0;
        cfg.lambda_tether_kg_per_m = 0.0;
        cfg.EA_design_N = 10000.0;
        cfg.damping_c_Ns_per_m = 0.0;
        cfg.altitude_km = 840.0;
        cfg.inclination_deg = 71.0;
        cfg.theta0_deg = 3.0;
        cfg.phi0_deg = 3.0;
        cfg.out_of_plane = true;
        cfg.controller = ControllerMode::Constant;
        cfg.dt_s = 0.2;
        cfg.sim_orbits = 16.0;
        cfg.const_current_a = current_for_eps(0.106, cfg.altitude_km, cfg.inclination_deg,
                                              cfg.m_parent_kg, cfg.m_tip_kg, cfg.tether_length_m);
        const SimResult r = run_tether_sim(cfg);

        // LOAD-BEARING qualitative check: diverges within O(10) orbits (NOT
        // the planar "bounded" behavior) via the cone-angle guard.
        CHECK(r.status == DivergeStatus::DivergedAngle);
        CHECK(std::isfinite(r.divergence_orbit));
        CHECK(r.divergence_orbit >= 2.0 && r.divergence_orbit <= 15.0);

        // o45 (T7's own cone-angle definition), same loose pre-registered band.
        CHECK(std::isfinite(r.o45_cone_orbit));
        CHECK(r.o45_cone_orbit >= 0.3 && r.o45_cone_orbit <= 2.5);
    }

    // 11. [DT-v2: 3D] free-libration frequency checks (D9): the two
    //     structural constants any correct 3D dumbbell reimplementation
    //     must reproduce, regardless of model-family offsets in the
    //     Lorentz-driven case above -- omega_pitch = sqrt(3)*n,
    //     omega_roll = 2*n. Measured via first-two-peaks period detection
    //     with I=0 (no Lorentz forcing at all, isolating pure
    //     GG+Coriolis). 5% tolerance (a from-scratch independent Python
    //     cross-check of this exact bead-model EOM matches theory to
    //     <0.1%, so 5% is a generous margin for a from-scratch C++
    //     reimplementation, not a tuned-to-pass band).
    {
        TetherConfig base;
        base.n_beads = 2;
        base.tether_length_m = 3000.0;
        base.m_parent_kg = 9000.0;
        base.m_tip_kg = 20.0;
        base.lambda_tether_kg_per_m = 0.0;
        base.EA_design_N = 10000.0;
        base.damping_c_Ns_per_m = 0.0;
        base.altitude_km = 840.0;
        base.inclination_deg = 71.0;
        base.controller = ControllerMode::Constant;
        base.const_current_a = 0.0;  // I=0: isolates free libration, no Lorentz term at all
        base.dt_s = 1.0;
        base.out_of_plane = true;

        const double n = tether_mean_motion_rad_s(base.altitude_km);
        const double t_orbit = 2.0 * kPi / n;
        const int max_steps = static_cast<int>(2.0 * t_orbit / base.dt_s);

        // Pitch: theta0=3deg, phi0=0 -- roll stays exactly 0 (no forcing to
        // excite it), so chord_angle_rad() cleanly tracks theta.
        {
            TetherConfig cfg = base;
            cfg.theta0_deg = 3.0;
            cfg.phi0_deg = 0.0;
            TetherSim sim(cfg);
            const double period = measure_period_s(sim, get_chord_angle, max_steps);
            CHECK(period > 0.0);
            const double expected = 2.0 * kPi / (std::sqrt(3.0) * n);
            CHECK(std::fabs(period - expected) / expected < 0.05);
        }
        // Roll: theta0=0, phi0=3deg.
        {
            TetherConfig cfg = base;
            cfg.theta0_deg = 0.0;
            cfg.phi0_deg = 3.0;
            TetherSim sim(cfg);
            const double period = measure_period_s(sim, get_roll_angle, max_steps);
            CHECK(period > 0.0);
            const double expected = 2.0 * kPi / (2.0 * n);
            CHECK(std::fabs(period - expected) / expected < 0.05);
        }
    }

    // 12. [DT-v2: 3D] same-PR planar regression, byte-identical config to
    //     CHECK 3's eps=0.106 case (out_of_plane left at its false
    //     default): re-confirms adding the 3D mode did not perturb the
    //     planar code path, pinned tightly against the committed
    //     generated/wp16_twin.csv Phase-1 figure (see also
    //     main_twin.cpp's run_planar_regression_check_3d, which emits this
    //     same check as a CSV row).
    {
        TetherConfig cfg;
        cfg.n_beads = 2;
        cfg.tether_length_m = 3000.0;
        cfg.m_parent_kg = 9000.0;
        cfg.m_tip_kg = 20.0;
        cfg.lambda_tether_kg_per_m = 0.0;
        cfg.EA_design_N = 10000.0;
        cfg.damping_c_Ns_per_m = 0.0;
        cfg.altitude_km = 840.0;
        cfg.inclination_deg = 71.0;
        cfg.theta0_deg = 3.0;
        cfg.controller = ControllerMode::Constant;
        cfg.dt_s = 0.2;
        cfg.sim_orbits = 5.0;
        cfg.const_current_a = current_for_eps(0.106, cfg.altitude_km, cfg.inclination_deg,
                                              cfg.m_parent_kg, cfg.m_tip_kg, cfg.tether_length_m);
        const SimResult r = run_tether_sim(cfg);
        CHECK(std::fabs(r.max_chord_angle_deg - 15.457219) < 1e-4);
    }

    std::printf("tether: all tests passed\n");
    return 0;
}
