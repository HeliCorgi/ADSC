// WP9a flight-software groundwork probe (software-only). TRL stays 4
// everywhere in this package; WP9 processor-in-the-loop (PIL) itself remains
// RESERVED, not started (docs/wp9_pil_plan.md, spec section 9). Nothing in
// this file claims real-time behavior, a timing guarantee, or flight
// readiness -- it is a workstation-only baseline.
//
// This drives the WP4-style combined control+estimation step (the loop body
// of run_estimated_sync, src/mission.cpp:285-379: measurement updates at
// their configured cadences, the tracking controller consuming ESTIMATES
// only, truth propagation, filter prediction) for a fixed number of steps,
// twice from an identical fixed-seed scenario, and checks:
//   (a) the number of heap allocations during the timed loop is IDENTICAL
//       across the two runs (a determinism regression guard) and stays
//       under a generous ceiling. The honest count is printed either way --
//       this file does not force a zero (docs/wp9_pil_plan.md section 5.1).
//   (b) the two runs produce bit-identical final state (R6: fixed seed, no
//       randomness source outside the single GaussianSource).
//   (c) per-step wall time (steady_clock) prints as p50/p95/max/mean, tagged
//       "CI hardware, NOT flight-representative - baseline only" and
//       asserted only against a very loose sanity bound so ordinary CI
//       variance cannot flake the build.
//
// Deliberately NOT reproduced here: the report-only NIS/NEES accumulation
// and the mission.cpp track_p() covariance-health probe (its implicit
// Eigen::MatrixXd conversion is test/report instrumentation, not part of
// the control path this file audits -- see docs/wp9_pil_plan.md 5.1).
//
// No generated/ artifact is written or committed by this test: timing is
// non-deterministic across machines and would break the byte-identity gate.
// Regenerate with `ctest -R wp9_flightlike`.
//
// Explicit return-1 checks (R4), fixed constants only, no randomness beyond
// the single fixed-seed GaussianSource (R6), no test-framework dependency
// (CHECK macro matches tests/test_decay.cpp).
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "adsc/mission.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

// ---------------------------------------------------------------------------
// File-local global operator new/delete override: counts heap allocations
// for the whole test_wp9_flightlike executable (each ctest binary is its own
// process, so this never touches any other test target). Fail-safe
// passthrough: on allocation failure this aborts rather than throwing, so no
// exception is ever thrown from these overrides.
// ---------------------------------------------------------------------------
static long g_alloc_count = 0;

void* operator new(std::size_t n) {
    ++g_alloc_count;
    void* p = std::malloc(n != 0 ? n : 1);
    if (p == nullptr) {
        std::abort();  // fail-safe: no exception thrown, ever
    }
    return p;
}

void* operator new[](std::size_t n) {
    return ::operator new(n);
}

void operator delete(void* p) noexcept {
    std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}

void operator delete[](void* p) noexcept {
    std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
    std::free(p);
}

// ---------------------------------------------------------------------------
// Probe harness.
// ---------------------------------------------------------------------------

// Warm-up steps run before the counted window opens (uncounted for both
// allocation and timing): first-touch effects only, not a claim about this
// phase. Measured steps are the actual WP9a probe window.
constexpr int kWarmupSteps  = 500;
constexpr int kMeasuredSteps = 2000;

// Generous allocation ceiling (per step, averaged over the measured window):
// if Eigen dynamics ever crept into this path we want this test to still
// pass comfortably while reporting the honest count, not to hunt for a tight
// bound. See docs/wp9_pil_plan.md section 6 (K1) for the kill-criteria this
// backstops.
constexpr long kAllocCeilingPerStep = 64;

// Very loose CI-variance-proof sanity bound on mean per-step wall time.
constexpr double kMeanWallTimeCeilingMs = 50.0;

// Flat final-state snapshot size (see the `put(...)` calls in run_probe):
// 4 (servicer q) + 3 (servicer w) + 4 (target q) + 3 (target w) +
// 4 (mekf q_own) + 4 (mekf q_rel) + 3 (mekf w_target) + 9 (mekf P_own 3x3) +
// 36 (mekf P_rel 6x6) + 6 (ekf state) + 36 (ekf covariance 6x6) +
// 3 (last commanded torque) = 115.
constexpr int kSnapSize = 115;

struct ProbeResult {
    long   allocs_used  = 0;
    int    filled_count = 0;
    double snap[kSnapSize] = {};
    double p50_ns  = 0.0;
    double p95_ns  = 0.0;
    double max_ns  = 0.0;
    double mean_ns = 0.0;
};

static ProbeResult run_probe() {
    using Clock = std::chrono::steady_clock;

    ProbeResult result;

    // Identical fixed scenario to tests/test_estimator.cpp (R6): same WP2
    // tumble + 40 deg offset start, same coasting relative orbit.
    const double deg2rad = kPi / 180.0;
    const Config cfg;
    const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d w_t0 =
        cfg.sync_target_rate_deg_s * deg2rad *
        Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
    const Eigen::Quaterniond q_c0(
        Eigen::AngleAxisd(40.0 * deg2rad,
                          Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

    GaussianSource rng(cfg.est_seed);

    RigidBody truth_target(cfg.target_inertia_diag.asDiagonal(), q_t0, w_t0);
    RigidBody truth_servicer(Eigen::Matrix3d::Identity() * cfg.base_inertia,
                             q_c0, Eigen::Vector3d::Zero());

    const CwModel cw =
        CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
    Vector6d truth_trans =
        cw.ellipse_state(SafetyEllipse{400.0, 400.0, 0.0}, 0.7);

    const Eigen::Matrix3d I_t     = truth_target.inertia();
    const Eigen::Matrix3d I_t_inv = I_t.inverse();
    const Eigen::Matrix3d I_servicer =
        Eigen::Matrix3d::Identity() * cfg.base_inertia;

    const double s_att = cfg.est_init_att_sigma_rad;
    const double s_wt  = cfg.est_init_wt_sigma_rad_s;

    Eigen::Matrix<double, 6, 6> P0_trans = Eigen::Matrix<double, 6, 6>::Zero();
    P0_trans.block<3, 3>(0, 0) = cfg.est_init_pos_sigma_m *
                                 cfg.est_init_pos_sigma_m *
                                 Eigen::Matrix3d::Identity();
    P0_trans.block<3, 3>(3, 3) = cfg.est_init_vel_sigma_m_s *
                                 cfg.est_init_vel_sigma_m_s *
                                 Eigen::Matrix3d::Identity();
    Vector6d x0_est = truth_trans;
    x0_est.head<3>() += rng.sample3(cfg.est_init_pos_sigma_m);
    x0_est.tail<3>() += rng.sample3(cfg.est_init_vel_sigma_m_s);
    TranslationEkf ekf(cw, x0_est, P0_trans);

    const Eigen::Quaterniond q_rel_true0 =
        (q_t0.conjugate() * q_c0).normalized();
    Eigen::Matrix<double, 6, 6> P0_rel = Eigen::Matrix<double, 6, 6>::Zero();
    P0_rel.block<3, 3>(0, 0) = s_att * s_att * Eigen::Matrix3d::Identity();
    P0_rel.block<3, 3>(3, 3) = s_wt * s_wt * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d e_own0 = rng.sample3(s_att);
    const Eigen::Vector3d e_rel0 = rng.sample3(s_att);
    const Eigen::Vector3d e_wt0  = rng.sample3(s_wt);
    AttitudeMekf mekf((q_c0 * quat_exp(e_own0)).normalized(),
                      s_att * s_att * Eigen::Matrix3d::Identity(),
                      (q_rel_true0 * quat_exp(e_rel0)).normalized(),
                      w_t0 + e_wt0, P0_rel, I_t);

    const SlidingModeController ctrl(cfg.sync_gains);

    const double dt = cfg.control_dt;
    const int st_every  = static_cast<int>(1.0 / (cfg.st_rate_hz * dt) + 0.5);
    const int vis_every = static_cast<int>(1.0 / (cfg.vision_rate_hz * dt) + 0.5);
    const int rng_every = static_cast<int>(1.0 / (cfg.ranging_rate_hz * dt) + 0.5);
    const double dt_ranging = rng_every * dt;

    // One combined control+estimation step (mirrors src/mission.cpp:285-379;
    // see the file header for what is deliberately left out). k_step is the
    // absolute step index since the start of this run (used for measurement
    // cadence only).
    const auto step = [&](int k_step) -> Eigen::Vector3d {
        if (k_step % st_every == 0) {
            const Eigen::Quaterniond q_st =
                (truth_servicer.attitude() *
                 quat_exp(rng.sample3(cfg.st_sigma_rad))).normalized();
            mekf.update_star_tracker(q_st, cfg.st_sigma_rad);
        }
        if (k_step % vis_every == 0) {
            const Eigen::Quaterniond q_rel_true =
                (truth_target.attitude().conjugate() *
                 truth_servicer.attitude()).normalized();
            const Eigen::Quaterniond q_vis =
                (q_rel_true * quat_exp(rng.sample3(cfg.vision_sigma_rad)))
                    .normalized();
            mekf.update_vision(q_vis, cfg.vision_sigma_rad);
        }
        if (k_step % rng_every == 0) {
            if (k_step > 0) {
                truth_trans = cw.propagate(truth_trans, dt_ranging);
                truth_trans.tail<3>() += rng.sample3(cfg.trans_vel_noise_m_s);
                ekf.predict(dt_ranging, cfg.trans_vel_noise_m_s);
            }
            const Eigen::Vector3d r_true = truth_trans.head<3>();
            const double z_range = r_true.norm() + cfg.range_bias_m +
                                   cfg.range_sigma_m * rng.sample();
            const Eigen::Vector3d z_los =
                r_true.normalized() +
                Eigen::Vector3d::Constant(cfg.los_bias) +
                rng.sample3(cfg.los_sigma);
            ekf.update(z_range, z_los, cfg.range_sigma_m, cfg.los_sigma);
        }

        const Eigen::Vector3d w_g =
            truth_servicer.rate() +
            Eigen::Vector3d::Constant(cfg.gyro_bias_rad_s) +
            rng.sample3(cfg.gyro_sigma_rad_s);

        EstimatedState est;
        est.q_own    = mekf.q_own();
        est.w_own    = w_g;
        est.q_target = (mekf.q_own() * mekf.q_rel().conjugate()).normalized();
        est.w_target = mekf.w_target();
        est.w_target_dot =
            I_t_inv * (-(mekf.w_target().cross(I_t * mekf.w_target())));
        est.rel_translation = ekf.state();

        const Eigen::Vector3d tau =
            ctrl.torque(I_servicer, est.q_own, est.w_own, est.q_target,
                        est.w_target, est.w_target_dot);

        truth_servicer.step(tau, dt);
        truth_target.step(I_t * rng.sample3(cfg.wt_disturb_sigma), dt);
        mekf.predict(w_g, dt, cfg.gyro_sigma_rad_s, cfg.wt_disturb_sigma);

        return tau;
    };

    // Warm-up: NOT counted for allocation or timing.
    for (int k = 0; k < kWarmupSteps; ++k) {
        step(k);
    }

    // Timed + allocation-counted phase: the actual WP9a measurement.
    double step_ns[kMeasuredSteps];
    double sum_ns = 0.0;
    Eigen::Vector3d last_tau = Eigen::Vector3d::Zero();
    const long allocs_before = g_alloc_count;
    for (int i = 0; i < kMeasuredSteps; ++i) {
        const int k = kWarmupSteps + i;
        const Clock::time_point t0 = Clock::now();
        last_tau = step(k);
        const Clock::time_point t1 = Clock::now();
        const double dur_ns =
            std::chrono::duration<double, std::nano>(t1 - t0).count();
        step_ns[i] = dur_ns;
        sum_ns += dur_ns;
    }
    result.allocs_used = g_alloc_count - allocs_before;

    double sorted_ns[kMeasuredSteps];
    std::memcpy(sorted_ns, step_ns, sizeof(sorted_ns));
    std::sort(sorted_ns, sorted_ns + kMeasuredSteps);
    result.p50_ns  = sorted_ns[kMeasuredSteps / 2];
    result.p95_ns  = sorted_ns[static_cast<int>(kMeasuredSteps * 0.95)];
    result.max_ns  = sorted_ns[kMeasuredSteps - 1];
    result.mean_ns = sum_ns / kMeasuredSteps;

    // Flat final-state snapshot (see kSnapSize's derivation comment above).
    int idx = 0;
    const auto put = [&](const double* src, int count) {
        std::memcpy(result.snap + idx, src,
                    static_cast<std::size_t>(count) * sizeof(double));
        idx += count;
    };
    put(truth_servicer.attitude().coeffs().data(), 4);
    put(truth_servicer.rate().data(), 3);
    put(truth_target.attitude().coeffs().data(), 4);
    put(truth_target.rate().data(), 3);
    put(mekf.q_own().coeffs().data(), 4);
    put(mekf.q_rel().coeffs().data(), 4);
    put(mekf.w_target().data(), 3);
    put(mekf.P_own().data(), 9);
    put(mekf.P_rel().data(), 36);
    put(ekf.state().data(), 6);
    put(ekf.covariance().data(), 36);
    put(last_tau.data(), 3);
    result.filled_count = idx;

    return result;
}

int main() {
    std::printf("wp9_flightlike: %d warm-up steps (uncounted) + %d measured "
               "steps per run, identical fixed-seed scenario both runs "
               "(R6)\n", kWarmupSteps, kMeasuredSteps);

    const ProbeResult a = run_probe();
    const ProbeResult b = run_probe();

    std::printf("wp9_flightlike: snapshot fields filled = %d (expect %d) "
               "run A, %d (expect %d) run B\n",
               a.filled_count, kSnapSize, b.filled_count, kSnapSize);
    CHECK(a.filled_count == kSnapSize);
    CHECK(b.filled_count == kSnapSize);

    // (a) Allocation count over the timed loop: deterministic across two
    // identical runs, and below a generous ceiling. The honest count prints
    // either way -- this test does not force a zero (docs/wp9_pil_plan.md
    // section 5.1: "the honest number, not a forced zero").
    const double allocs_per_step_a =
        static_cast<double>(a.allocs_used) / kMeasuredSteps;
    std::printf("wp9_flightlike: heap allocations over the timed loop = "
               "%ld (run A), %ld (run B); %.4f/step average; ceiling %ld/"
               "step average (regenerate: ctest -R wp9_flightlike)\n",
               a.allocs_used, b.allocs_used, allocs_per_step_a,
               kAllocCeilingPerStep);
    CHECK(a.allocs_used == b.allocs_used);
    CHECK(a.allocs_used <= kAllocCeilingPerStep * kMeasuredSteps);
    CHECK(b.allocs_used <= kAllocCeilingPerStep * kMeasuredSteps);

    // (b) Bit-identical final state across two identical runs (R6
    // determinism regression guard).
    const bool bit_identical =
        std::memcmp(a.snap, b.snap, sizeof(a.snap)) == 0;
    std::printf("wp9_flightlike: final-state bit-identical across two runs "
               "= %s\n", bit_identical ? "yes" : "no");
    CHECK(bit_identical);

    // (c) Per-step wall time: CI-hardware baseline only, loose sanity bound
    // so ordinary CI variance cannot flake this test. This is NOT a WCET
    // measurement and NOT a real-time claim (docs/wp9_pil_plan.md section
    // 3.3 is the frozen WCET method for the future hardware-gated WP9).
    std::printf("wp9_flightlike: per-step wall time (CI hardware, NOT "
               "flight-representative - baseline only): p50=%.4f ms "
               "p95=%.4f ms max=%.4f ms mean=%.4f ms (regenerate: ctest -R "
               "wp9_flightlike)\n",
               a.p50_ns / 1.0e6, a.p95_ns / 1.0e6, a.max_ns / 1.0e6,
               a.mean_ns / 1.0e6);
    CHECK(a.mean_ns / 1.0e6 < kMeanWallTimeCeilingMs);

    std::printf("wp9_flightlike: all tests passed\n");
    return 0;
}
