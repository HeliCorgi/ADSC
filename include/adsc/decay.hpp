#pragma once

#include <cmath>

#include "adsc/relmotion.hpp"  // kEarthMu, kEarthRadius

namespace adsc {

// ============================================================================
// Kit deorbit-decay trades (WP3)
// ----------------------------------------------------------------------------
// A quasi-circular drag-decay model plus a first-order EDT placeholder, used to
// produce the sail-area x altitude x solar-activity -> decay-time trade tables.
// The honest headline result (sail-only on a multi-tonne stage at ~840 km can
// exceed the 25-year guideline) is a deliverable, not a bug: it is what
// motivates the electrodynamic-tether branch and open trade T1.
// ============================================================================

// A class-parameter debris target (D2): mass / altitude / inclination presets
// that let each potential adopter see its own debris. Values are class-level
// figures from public debris-ranking literature (cite when filling C/D), never
// specific catalog object IDs (D11 / R13).
struct DebrisCatalog {
    const char* name;
    double mass_kg;
    double altitude_km;
    double inclination_deg;
    bool   placeholder;   // true = mass/altitude not yet filled from literature
};

DebrisCatalog catalog_A();  // SL-16 / Zenit-2 second-stage class
DebrisCatalog catalog_B();  // SL-8 / Kosmos-3M second-stage class
DebrisCatalog catalog_C();  // CZ upper-stage class (PLACEHOLDER)
DebrisCatalog catalog_D();  // US Delta-class stage (PLACEHOLDER)

// Piecewise-exponential atmospheric density [kg/m^3] at geodetic altitude [m].
// Base densities and scale heights are the standard exponential model from
// Vallado, "Fundamentals of Astrodynamics and Applications" (4th ed.,
// Table 8-4). solar_factor multiplies the whole profile: a single factor is a
// deliberately coarse proxy for the solar cycle (the real swing is strongly
// altitude-dependent — roughly an order of magnitude near 800 km), which is
// exactly why every decay figure below is reported as a solar-min..solar-max
// RANGE and never a point value (T4).
double atmospheric_density(double altitude_m, double solar_factor);

// Quasi-circular drag-decay integrator for the spec's model
//   da/dt = -rho(h) * Cd * (A/m) * sqrt(mu * a)
// Integrates from a0_m down to a_stop_m and returns the elapsed time [s].
// Templated on a density-of-altitude[m] callable so the constant-density
// closed-form check can drive the same integrator directly. Quadrature is
// trapezoidal in `a`: the integrand 1/(rho*Cd*(A/m)*sqrt(mu*a)) is smooth and
// the decay time is dominated by the slow, high-altitude leg near a0.
template <typename DensityFn>
double integrate_decay_seconds(double a0_m, double a_stop_m, double cd,
                               double area_over_mass, DensityFn density_at_altitude,
                               int steps = 6000) {
    if (a0_m <= a_stop_m || steps < 1 || cd <= 0.0 || area_over_mass <= 0.0) {
        return 0.0;
    }
    const double da = (a0_m - a_stop_m) / steps;
    const auto integrand = [&](double a) {
        const double rho = density_at_altitude(a - kEarthRadius);
        return 1.0 / (rho * cd * area_over_mass * std::sqrt(kEarthMu * a));
    };
    double sum = 0.5 * (integrand(a_stop_m) + integrand(a0_m));
    for (int i = 1; i < steps; ++i) {
        sum += integrand(a_stop_m + static_cast<double>(i) * da);
    }
    return sum * da;
}

// Sail-only decay time [years]: a catalog target fitted with a drag sail of
// sail_area_m2 and an installed kit of kit_mass_kg (added to the target mass),
// decaying from the catalog altitude to stop_altitude_km at the given solar
// factor.
double sail_decay_years(const DebrisCatalog& target, double sail_area_m2,
                        double kit_mass_kg, double cd, double stop_altitude_km,
                        double solar_factor);

// Sail area [m^2] that brings the target down within target_years at the given
// solar factor. Exact: decay time is inversely proportional to A/m in this
// model, so one reference evaluation scales to any target time.
double area_for_target_years(const DebrisCatalog& target, double target_years,
                             double kit_mass_kg, double cd, double stop_altitude_km,
                             double solar_factor);

// First-order parametric EDT knob (R10 placeholder): for an effective
// along-track semi-major-axis decay rate delta_a_per_day [m/day], the deorbit
// time [days] is the altitude drop over that rate. This is a PARAMETRIC STUDY
// axis (performance parameter vs deorbit time), NOT a performance claim for any
// specific tether.
double edt_deorbit_days(double a0_m, double a_stop_m, double delta_a_per_day_m);

}  // namespace adsc
