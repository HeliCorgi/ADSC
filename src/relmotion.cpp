#include "adsc/relmotion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adsc {

CwModel CwModel::from_orbit(double semi_major_axis_m, double mu) {
    return CwModel(std::sqrt(mu / (semi_major_axis_m * semi_major_axis_m *
                                   semi_major_axis_m)));
}

Matrix6d CwModel::stm(double t) const {
    const double n = n_;
    const double s = std::sin(n * t);
    const double c = std::cos(n * t);

    Matrix6d phi = Matrix6d::Zero();

    // Position from initial position (Phi_rr).
    phi(0, 0) = 4.0 - 3.0 * c;
    phi(1, 0) = 6.0 * (s - n * t);
    phi(1, 1) = 1.0;
    phi(2, 2) = c;

    // Position from initial velocity (Phi_rv).
    phi(0, 3) = s / n;
    phi(0, 4) = 2.0 * (1.0 - c) / n;
    phi(1, 3) = 2.0 * (c - 1.0) / n;
    phi(1, 4) = (4.0 * s - 3.0 * n * t) / n;
    phi(2, 5) = s / n;

    // Velocity from initial position (Phi_vr).
    phi(3, 0) = 3.0 * n * s;
    phi(4, 0) = 6.0 * n * (c - 1.0);
    phi(5, 2) = -n * s;

    // Velocity from initial velocity (Phi_vv).
    phi(3, 3) = c;
    phi(3, 4) = 2.0 * s;
    phi(4, 3) = -2.0 * s;
    phi(4, 4) = 4.0 * c - 3.0;
    phi(5, 5) = c;

    return phi;
}

Vector6d CwModel::propagate(const Vector6d& x0, double t) const {
    return stm(t) * x0;
}

Vector6d CwModel::derivative(const Vector6d& x, const Eigen::Vector3d& accel) const {
    const double n = n_;
    Vector6d dx;
    dx(0) = x(3);
    dx(1) = x(4);
    dx(2) = x(5);
    dx(3) = 3.0 * n * n * x(0) + 2.0 * n * x(4) + accel.x();
    dx(4) = -2.0 * n * x(3) + accel.y();
    dx(5) = -n * n * x(2) + accel.z();
    return dx;
}

Vector6d CwModel::propagate_rk4(const Vector6d& x0, double t, double dt,
                               const Eigen::Vector3d& accel) const {
    Vector6d x = x0;
    double remaining = t;
    while (remaining > 1e-12) {
        const double h = std::min(dt, remaining);
        const Vector6d k1 = derivative(x, accel);
        const Vector6d k2 = derivative(x + 0.5 * h * k1, accel);
        const Vector6d k3 = derivative(x + 0.5 * h * k2, accel);
        const Vector6d k4 = derivative(x + h * k3, accel);
        x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        remaining -= h;
    }
    return x;
}

Vector6d CwModel::ellipse_state(const SafetyEllipse& e, double theta) const {
    const double n = n_;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    Vector6d x;
    x(0) = e.rho * ct;
    x(1) = -2.0 * e.rho * st + e.y_center;
    x(2) = e.z_amp * ct;
    x(3) = -e.rho * n * st;
    x(4) = -2.0 * e.rho * n * ct;   // == -2 n * x(0): drift-free by construction
    x(5) = -e.z_amp * n * st;
    return x;
}

double SafetyEllipse::min_range() const {
    double best = std::numeric_limits<double>::max();
    const int samples = 1440;  // 0.25 deg resolution
    for (int i = 0; i < samples; ++i) {
        const double th = 2.0 * kPi * static_cast<double>(i) / samples;
        const double x = rho * std::cos(th);
        const double y = -2.0 * rho * std::sin(th) + y_center;
        const double z = z_amp * std::cos(th);
        const double r = std::sqrt(x * x + y * y + z * z);
        best = std::min(best, r);
    }
    return best;
}

double bounded_coast_min_range(double x0, double y0, double z0,
                               double vx, double vz, double n) {
    double best = std::numeric_limits<double>::max();
    const int samples = 1440;  // 0.25 deg resolution, matches SafetyEllipse::min_range
    for (int i = 0; i < samples; ++i) {
        const double nt = 2.0 * kPi * static_cast<double>(i) / samples;
        const double c = std::cos(nt);
        const double s = std::sin(nt);
        const double x = x0 * c + (vx / n) * s;
        const double y = y0 - 2.0 * x0 * s + 2.0 * (vx / n) * (c - 1.0);
        const double z = z0 * c + (vz / n) * s;
        const double r = std::sqrt(x * x + y * y + z * z);
        best = std::min(best, r);
    }
    return best;
}

std::vector<SafetyEllipse> approach_corridor(double rho_far, double rho_near,
                                             double keep_out, int n_holds) {
    std::vector<SafetyEllipse> holds;
    if (n_holds < 1) return holds;

    for (int i = 0; i < n_holds; ++i) {
        const double frac =
            (n_holds == 1) ? 0.0 : static_cast<double>(i) / (n_holds - 1);
        const double rho = rho_far + frac * (rho_near - rho_far);
        const SafetyEllipse e{rho, rho, 0.0};  // z_amp = rho -> min = sqrt(2)*rho
        if (e.min_range() > keep_out) holds.push_back(e);
    }
    return holds;
}

}  // namespace adsc
