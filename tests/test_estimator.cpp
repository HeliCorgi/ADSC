// WP4 estimator tests: covariance health (symmetry + positive definiteness),
// the NEES/NIS filter-consistency watchdog, and the estimate-driven WP2
// acceptance. Explicit return-1 checks (R4); all randomness from the single
// fixed-seed GaussianSource inside run_estimated_sync (R6, bit-stable).
//
// The consistency watchdog is the estimator's analogue of WP2's
// feedforward-honesty test: an estimator whose Q or R is inflated "to make
// the acceptance pass" produces innovations that are too small for its own
// covariance — the time-averaged NIS falls below the chi-square band and this
// test fails. NIS uses the strict single-run bound (innovations of a
// consistent filter are white, so the N-sample average is chi2(N*d)/N).
// NEES errors are correlated in time, so its band uses an effective sample
// size N_eff = N/10 (documented correlation allowance); both use z = 3
// (99.7%) Wilson-Hilferty quantiles so a passing filter is robust to
// seed luck while a Q inflated by even a few x still fails clearly.
#include <cmath>
#include <cstdio>

#include "adsc/mission.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

// Wilson-Hilferty chi-square quantile approximation for k dof at z sigmas.
static double chi2_quantile(double k, double z) {
    const double a = 2.0 / (9.0 * k);
    const double b = 1.0 - a + z * std::sqrt(a);
    return k * b * b * b;
}

// Two-sided band for the MEAN of N chi2(d)/1 samples with effective sample
// count n_eff (n_eff == N for white sequences like NIS).
struct Band {
    double lo, hi;
};
static Band mean_band(double d, double n_eff, double z) {
    return {chi2_quantile(d * n_eff, -z) / n_eff,
            chi2_quantile(d * n_eff, z) / n_eff};
}

int main() {
    const double deg2rad = kPi / 180.0;

    // Fixed WP2 scenario + a coasting relative orbit for the translation EKF —
    // identical constants to the adsc_sim scenario 7 demo (R6).
    const Config cfg;
    const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                 Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
    const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
        40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));
    const CwModel cw =
        CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
    const Vector6d x_trans0 = cw.ellipse_state(SafetyEllipse{400.0, 400.0, 0.0}, 0.7);

    const EstimatedSyncReport rep =
        run_estimated_sync(cfg, q_t0, w_t0, q_c0, x_trans0, 120.0);

    // 1. Acceptance (spec section 5 WP4): the WP2 sync criteria hold on the
    //    TRUTH state while the controller is driven by estimates under noise.
    std::printf("estimator: est-driven sync at %.2f s; post-dwell "
                "max |w_rel| %.4f deg/s, max att %.4f deg\n",
                rep.sync.sync_time_s, rep.sync.max_rate_err_deg_s,
                rep.sync.max_att_err_deg);
    CHECK(rep.sync.synced);
    CHECK(rep.sync.sync_time_s < 90.0);
    CHECK(rep.sync.max_rate_err_deg_s < cfg.sync_rate_tol_deg_s);
    CHECK(rep.sync.max_att_err_deg < cfg.sync_att_tol_deg);

    // 2. Covariance health across every filter over the whole run: symmetric
    //    (Joseph form + explicit symmetrization) and positive definite.
    std::printf("estimator: P min eigenvalue %.3e, max asymmetry %.3e\n",
                rep.p_min_eig, rep.p_max_asym);
    CHECK(rep.p_min_eig > 0.0);
    CHECK(rep.p_max_asym < 1e-12);

    // 3. Consistency watchdog. Sample counts must be what the cadences imply
    //    for the 80 s stats window, then the chi-square bands apply.
    std::printf("estimator: NIS  trans %.3f (d=4, N=%d)  st %.3f (d=3, N=%d)  "
                "vis %.3f (d=3, N=%d)\n",
                rep.nis_trans_mean, rep.n_trans, rep.nis_st_mean, rep.n_st,
                rep.nis_vis_mean, rep.n_vis);
    std::printf("estimator: NEES trans %.3f (n=6)  own %.3f (n=3)  "
                "rel %.3f (n=6)\n",
                rep.nees_trans_mean, rep.nees_own_mean, rep.nees_rel_mean);
    CHECK(rep.n_trans >= 700);   // ~800 expected (80 s at 10 Hz)
    CHECK(rep.n_st >= 350);      // ~400 expected (80 s at 5 Hz)
    CHECK(rep.n_vis >= 140);     // ~160 expected (80 s at 2 Hz)

    {
        const Band b = mean_band(4.0, rep.n_trans, 3.0);   // NIS, white
        CHECK(rep.nis_trans_mean > b.lo && rep.nis_trans_mean < b.hi);
    }
    {
        const Band b = mean_band(3.0, rep.n_st, 3.0);
        CHECK(rep.nis_st_mean > b.lo && rep.nis_st_mean < b.hi);
    }
    {
        const Band b = mean_band(3.0, rep.n_vis, 3.0);
        CHECK(rep.nis_vis_mean > b.lo && rep.nis_vis_mean < b.hi);
    }
    {
        const Band b = mean_band(6.0, rep.n_trans / 10.0, 3.0);  // NEES, N_eff
        CHECK(rep.nees_trans_mean > b.lo && rep.nees_trans_mean < b.hi);
    }
    {
        const Band b = mean_band(3.0, rep.n_st / 10.0, 3.0);
        CHECK(rep.nees_own_mean > b.lo && rep.nees_own_mean < b.hi);
    }
    {
        const Band b = mean_band(6.0, rep.n_vis / 10.0, 3.0);
        CHECK(rep.nees_rel_mean > b.lo && rep.nees_rel_mean < b.hi);
    }

    // 4. Estimation-error sanity (generous physical bounds; the exact values
    //    are demo/README material, regenerated by adsc_sim scenario 7).
    std::printf("estimator: RMS own att %.4f deg, rel att %.4f deg, "
                "w_t %.5f deg/s, pos %.3f m, vel %.5f m/s\n",
                rep.rms_att_own_deg, rep.rms_att_rel_deg, rep.rms_wt_deg_s,
                rep.rms_pos_m, rep.rms_vel_m_s);
    CHECK(rep.rms_att_own_deg < 0.05);
    CHECK(rep.rms_att_rel_deg < 0.5);
    CHECK(rep.rms_wt_deg_s < 0.05);
    CHECK(rep.rms_pos_m < 1.0);
    CHECK(rep.rms_vel_m_s < 0.05);

    std::printf("estimator: all tests passed\n");
    return 0;
}
