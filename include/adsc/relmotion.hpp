#pragma once

#include <vector>

#include <Eigen/Dense>

namespace adsc {

// ============================================================================
// Relative orbital motion (WP1)
// ----------------------------------------------------------------------------
// Frame convention (R8): LVLH ("Hill") frame centred on the *target*, which is
// assumed to be on a circular orbit.
//     x = radial      (out from Earth, along the target position vector)
//     y = along-track  (target velocity direction)
//     z = cross-track  (orbit normal)
// All quantities are SI (m, m/s, s, rad). The relative state is the 6-vector
//     X = [x, y, z, vx, vy, vz].
//
// Dynamics are the Clohessy-Wiltshire (Hill) linearisation about the circular
// target orbit (mean motion n = sqrt(mu / a^3)):
//     x_ddot - 2 n y_dot - 3 n^2 x = a_x
//     y_ddot + 2 n x_dot           = a_y
//     z_ddot + n^2 z               = a_z
// where a = (a_x, a_y, a_z) is an external (control) acceleration.
// ============================================================================

// Fixed-size relative state / transition types.
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Physical constants (not placeholders: standard geodetic values).
constexpr double kEarthMu     = 3.986004418e14;  // Earth GM [m^3/s^2] (WGS-84)
constexpr double kEarthRadius = 6378137.0;       // Earth equatorial radius [m]
constexpr double kPi          = 3.14159265358979323846;

// Convenience views on a relative state (kept as free helpers so callers never
// slice the 6-vector by hand).
inline Eigen::Vector3d rel_pos(const Vector6d& x)  { return x.head<3>(); }
inline Eigen::Vector3d rel_vel(const Vector6d& x)  { return x.tail<3>(); }
inline double          rel_range(const Vector6d& x){ return x.head<3>().norm(); }

// A passively-safe natural-motion trajectory ("safety ellipse"): a drift-free
// Clohessy-Wiltshire relative orbit. In-plane it is the classic 2:1 ellipse
// (radial semi-amplitude rho, along-track 2*rho); the cross-track oscillation
// (amplitude z_amp) is carried in phase with the radial term so that when the
// in-plane range is smallest the 3D range is lifted by the cross-track offset.
// The drift-free condition vy0 = -2 n x0 (no secular along-track walk-off) is
// satisfied at every phase by construction.
struct SafetyEllipse {
    double rho;       // in-plane radial semi-amplitude [m]
    double z_amp;     // cross-track amplitude [m]
    double y_center;  // along-track offset of the ellipse centre [m]

    // Minimum 3D range to the target over one full period [m]. Computed by
    // dense sampling of the closed-form ellipse, so it is correct for any
    // y_center (not just the centred case).
    double min_range() const;
};

// Clohessy-Wiltshire relative dynamics about a circular target orbit.
class CwModel {
public:
    // Construct directly from the target mean motion n [rad/s].
    explicit CwModel(double mean_motion) : n_(mean_motion) {}

    // Construct from the target circular-orbit semi-major axis a [m].
    static CwModel from_orbit(double semi_major_axis_m, double mu = kEarthMu);

    double n() const      { return n_; }
    double period() const { return 2.0 * kPi / n_; }  // orbital period [s]

    // Analytic state-transition matrix Phi(t) for a coast (a = 0).
    Matrix6d stm(double t) const;

    // Analytic coast propagation: X(t) = Phi(t) X0.
    Vector6d propagate(const Vector6d& x0, double t) const;

    // State derivative under a constant control acceleration.
    Vector6d derivative(const Vector6d& x,
                        const Eigen::Vector3d& accel = Eigen::Vector3d::Zero()) const;

    // Fixed-step RK4 propagation (control acceleration held constant over the
    // interval). Cross-validated against stm()/propagate() in the unit tests.
    Vector6d propagate_rk4(const Vector6d& x0, double t, double dt,
                           const Eigen::Vector3d& accel = Eigen::Vector3d::Zero()) const;

    // State on a safety ellipse at phase angle theta [rad].
    Vector6d ellipse_state(const SafetyEllipse& e, double theta) const;

private:
    double n_;  // mean motion [rad/s]
};

// Generate a nominal approach corridor as a sequence of passively-safe hold
// ellipses at decreasing radial standoff, from rho_far down to rho_near. Only
// holds whose free-motion minimum range exceeds keep_out are returned, so every
// point of the resulting corridor is passively safe (D5). z_amp is set equal to
// rho for each hold, giving min_range = sqrt(2)*rho for the centred case.
std::vector<SafetyEllipse> approach_corridor(double rho_far, double rho_near,
                                             double keep_out, int n_holds);

}  // namespace adsc
