#pragma once
#include <atomic>
#include <array>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <type_traits>
#include <Eigen/Dense>

struct FuelCell {
    double value;
    uint32_t version;
    uint32_t crc;
};

static_assert(std::is_trivially_copyable<FuelCell>::value, "FuelCell must be trivially copyable");

class ADSC_Core {
private:
    std::array<std::atomic<FuelCell>, 3> fuel_cells;

    static uint32_t compute_crc(double v, uint32_t ver);
    static bool verify_crc(const FuelCell& c);
    static bool nearly_equal(double a, double b, double eps = 1e-9);
    static double median(double a, double b, double c);

    // 捕獲パラメータ
    const double max_v_rel = 0.55; // [m/s] 捕獲閾値、95%成功率目安

public:
    double mass_total;
    Eigen::Matrix3d inertia_tensor;

public:
    ADSC_Core();
    double get_fuel();
    void set_fuel(double f);

    void post_capture_stabilization(bool captured, double debris_mass, const Eigen::Vector3d& v_rel_vec);
};
