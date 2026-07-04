#include "adsc/propagation.hpp"

#include <algorithm>
#include <cmath>

#include "adsc/decay.hpp"  // atmospheric_density (R1 reuse, L2 drag term)

namespace adsc {

namespace {

// Inertial (ECI) two-body [+J2] [+drag] acceleration for ONE craft's state
// (r, v), given its own ballistic coefficient bc = Cd*(A/m) [m^2/kg]. See the
// propagation.hpp file header for the J2 formula (ECI frame, z along the
// polar/rotation axis -- standard Vallado/Curtis form) and the drag formula
// (co-rotating atmosphere at kEarthOmega about the SAME polar axis).
Vector6d inertial_derivative(const Vector6d& x, bool enable_j2, bool enable_drag,
                             double bc, double solar_factor) {
    const Eigen::Vector3d r = x.head<3>();
    const Eigen::Vector3d v = x.tail<3>();
    const double rn = r.norm();

    Eigen::Vector3d a = -(kEarthMu / (rn * rn * rn)) * r;

    if (enable_j2) {
        const double z_over_r = r.z() / rn;
        const double f = -1.5 * kEarthJ2 * kEarthMu * kEarthRadius * kEarthRadius /
                         (rn * rn * rn * rn * rn);
        a.x() += f * r.x() * (1.0 - 5.0 * z_over_r * z_over_r);
        a.y() += f * r.y() * (1.0 - 5.0 * z_over_r * z_over_r);
        a.z() += f * r.z() * (3.0 - 5.0 * z_over_r * z_over_r);
    }

    if (enable_drag) {
        const Eigen::Vector3d omega_e(0.0, 0.0, kEarthOmega);
        const Eigen::Vector3d v_atm = v - omega_e.cross(r);
        const double alt_m = rn - kEarthRadius;
        const double rho = atmospheric_density(alt_m, solar_factor);
        a += -0.5 * rho * v_atm.norm() * bc * v_atm;
    }

    Vector6d dx;
    dx.head<3>() = v;
    dx.tail<3>() = a;
    return dx;
}

// Fixed-step RK4 (SAME coefficients as CwModel::propagate_rk4, relmotion.cpp)
// for one craft's inertial state over a single step of size h.
Vector6d inertial_rk4_step(const Vector6d& x, double h, bool enable_j2,
                           bool enable_drag, double bc, double solar_factor) {
    const Vector6d k1 = inertial_derivative(x, enable_j2, enable_drag, bc, solar_factor);
    const Vector6d k2 = inertial_derivative(x + 0.5 * h * k1, enable_j2, enable_drag, bc, solar_factor);
    const Vector6d k3 = inertial_derivative(x + 0.5 * h * k2, enable_j2, enable_drag, bc, solar_factor);
    const Vector6d k4 = inertial_derivative(x + h * k3, enable_j2, enable_drag, bc, solar_factor);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// The target's instantaneous LVLH triad (see propagation.hpp file header):
// C's rows are xhat/yhat/zhat (ECI -> LVLH rotation matrix), omega_mag is the
// exact osculating in-plane rotation rate h/r^2.
struct LvlhFrame {
    Eigen::Matrix3d C;
    double omega_mag;
};

LvlhFrame lvlh_frame(const Vector6d& x_target_eci) {
    const Eigen::Vector3d r = x_target_eci.head<3>();
    const Eigen::Vector3d v = x_target_eci.tail<3>();
    const Eigen::Vector3d h = r.cross(v);
    const Eigen::Vector3d xhat = r.normalized();
    const Eigen::Vector3d zhat = h.normalized();
    const Eigen::Vector3d yhat = zhat.cross(xhat);

    LvlhFrame f;
    f.C.row(0) = xhat.transpose();
    f.C.row(1) = yhat.transpose();
    f.C.row(2) = zhat.transpose();
    f.omega_mag = h.norm() / (r.norm() * r.norm());
    return f;
}

// Target circular-orbit ECI initial condition at u = 0, RAAN = 0 (see
// propagation.hpp file header derivation).
Vector6d target_initial_state(double alt_km, double incl_deg) {
    const double deg2rad = kPi / 180.0;
    const double a = kEarthRadius + alt_km * 1000.0;
    const double i = incl_deg * deg2rad;
    const double n = std::sqrt(kEarthMu / (a * a * a));
    const double v0 = a * n;  // circular speed

    Vector6d x;
    x << a, 0.0, 0.0,
         0.0, v0 * std::cos(i), v0 * std::sin(i);
    return x;
}

}  // namespace

double fidelity_coast_min_range_terms(bool enable_j2, bool enable_drag,
                                      const Vector6d& x_rel_lvlh6,
                                      double alt_km, double incl_deg,
                                      double horizon_s, double dt_s,
                                      double solar_factor,
                                      double bc_chaser, double bc_target) {
    Vector6d x_target = target_initial_state(alt_km, incl_deg);

    // Place the chaser in ECI at t = 0 from the given LVLH relative state
    // (the ONE documented conversion, propagation.hpp file header).
    const LvlhFrame f0 = lvlh_frame(x_target);
    const Eigen::Vector3d r_rel0 = x_rel_lvlh6.head<3>();
    const Eigen::Vector3d v_rel0 = x_rel_lvlh6.tail<3>();
    const Eigen::Vector3d omega0(0.0, 0.0, f0.omega_mag);  // LVLH-frame components (along zhat)
    const Eigen::Vector3d v_rel_transport = v_rel0 + omega0.cross(r_rel0);

    Vector6d x_chaser;
    x_chaser.head<3>() = x_target.head<3>() + f0.C.transpose() * r_rel0;
    x_chaser.tail<3>() = x_target.tail<3>() + f0.C.transpose() * v_rel_transport;

    double min_range = r_rel0.norm();
    double t = 0.0;
    while (t < horizon_s) {
        x_target = inertial_rk4_step(x_target, dt_s, enable_j2, enable_drag,
                                     bc_target, solar_factor);
        x_chaser = inertial_rk4_step(x_chaser, dt_s, enable_j2, enable_drag,
                                     bc_chaser, solar_factor);
        t += dt_s;

        // Difference into the target's FRESH instantaneous LVLH frame (no
        // accumulated frame-rotation error -- see file header).
        const LvlhFrame f = lvlh_frame(x_target);
        const Eigen::Vector3d r_rel = f.C * (x_chaser.head<3>() - x_target.head<3>());
        min_range = std::min(min_range, r_rel.norm());
    }
    return min_range;
}

double fidelity_coast_min_range(FidelityLevel level, const Vector6d& x_rel_lvlh6,
                                double alt_km, double incl_deg,
                                double horizon_s, double dt_s, double solar_factor,
                                double bc_chaser, double bc_target) {
    if (level == FidelityLevel::L0_CW) {
        // Bit-identical path: the ORIGINAL CW RK4 sweep, untouched (R1).
        const CwModel cw = CwModel::from_orbit(kEarthRadius + alt_km * 1000.0);
        Vector6d x = x_rel_lvlh6;
        double min_range = rel_range(x);
        double t = 0.0;
        while (t < horizon_s) {
            x = cw.propagate_rk4(x, dt_s, dt_s);
            min_range = std::min(min_range, rel_range(x));
            t += dt_s;
        }
        return min_range;
    }

    const bool enable_drag = (level == FidelityLevel::L2_J2_DRAG);
    return fidelity_coast_min_range_terms(/*enable_j2=*/true, enable_drag,
                                          x_rel_lvlh6, alt_km, incl_deg,
                                          horizon_s, dt_s, solar_factor,
                                          bc_chaser, bc_target);
}

}  // namespace adsc
