#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <Eigen/Dense>
#include <iostream>

struct FuelCell {
    double   value;
    uint32_t version;
    uint32_t crc;
};

static_assert(std::is_trivially_copyable<FuelCell>::value, "FuelCell must be trivially copyable");

class ADSC_Core {
private:
    std::array<std::atomic<FuelCell>, 3> fuel_cells;

    static uint32_t compute_crc(double v, uint32_t ver);
    static bool     verify_crc(const FuelCell& c);
    static bool     nearly_equal(double a, double b, double eps = 1e-9);
    static double   median(double a, double b, double c);

    // v1.21 Reality Final: 7-state MAG-Adaptive SR-UKF (computationally feasible)
    static constexpr int UKF_N = 7;
    Eigen::Matrix<double, UKF_N, 1> ukf_x;
    Eigen::Matrix<double, UKF_N, UKF_N> ukf_S;

    Eigen::Vector3d last_b_meas, last_b_model;

    double operation_time_sec = 0.0;
    double pcm_remaining_joules = 5000.0;
    static constexpr double PCM_MAX_JOULES = 5000.0;
    static constexpr double MAX_SAFE_TIME  = 14400.0;   // 4 hours

    static constexpr double DACS_DEADBAND_RAD = 0.015;
    static constexpr double DACS_MIN_IMPULSE  = 0.5;

    const double deorbit_fuel_reserve = 1.2;
    static constexpr double regularization_epsilon = 1e-6;

public:
    double          mass_total     = 27.2;
    Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity() * 0.55;
    static constexpr double max_v_rel = 0.15;           // based on RemoveDEBRIS demonstrated values 0.075–0.12 m/s with safety margin

    ADSC_Core();

    double get_fuel();
    void   set_fuel(double f);

    Eigen::Vector3d compute_safe_abort(const Eigen::Vector3d& r_rel, const Eigen::Vector3d& v_rel);

    void magnav_fallback_update(const Eigen::Vector3d& b_meas, const Eigen::Vector3d& b_model_igrf);

    void optimized_dacs_logic(const Eigen::Vector3d& attitude_error_rad);

    void update_disturbance_state(double measured_disturbance);

    bool check_thermal_saturation();

    bool deorbit_protocol(bool ground_human_approval = true);

    void regularized_inertia_update(const Eigen::Vector3d& r_attach, double debris_mass);

    void post_capture_stabilization(bool captured,
                                    double debris_mass,
                                    const Eigen::Vector3d& r_attach,
                                    const Eigen::Vector3d& r_rel,
                                    const Eigen::Vector3d& v_rel_vec);
};
