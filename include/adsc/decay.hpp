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
DebrisCatalog catalog_C();  // Envisat-class massive SSO payload (WP13, cited)
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

// ============================================================================
// Electrodynamic-tether (EDT) physics core (WP13)
// ----------------------------------------------------------------------------
// Aligned-dipole v x B model for a radial (nadir-aligned) bare tether. This is
// a NEW model alongside edt_deorbit_days above (kept byte-for-byte, R15 pin) --
// it does not replace or edit that legacy parametric knob or any sail/
// atmosphere function in this file. See wp13-edt-derivation.md
// for the full eta(i) derivation and wp13-literature.md for
// citations (Sanmartin, Martinez-Sanchez & Ahedo 1993 OML bare-tether
// baseline; BETs FP7 GA 262972 tape-tether reference geometry; SPENVIS
// centred-dipole field; Pelaez et al. 2000 EDT libration/dynamic-instability
// finding). Every output is reported as a MANDATORY [conservative,
// optimistic] band -- never a point value (spec:247-249).
// ============================================================================

struct EdtConfig {
    double tether_length_m = 3000.0;    // BETs FP7 tape-tether reference length [m] (2 km bare + 1 km inert; thebetsproject.com / oa.upm.es/39287, cited wp13-literature.md Topic 1b)
    double kit_mass_kg = 20.0;          // PLACEHOLDER EDT kit mass (tether + deployer + electronics) [kg]
    double avg_current_a = 2.0;         // PLACEHOLDER average bare-tether collected current, BETs-scale [A]
    double eta_libration = 0.75;        // PLACEHOLDER libration/duty-cycle efficiency penalty [-] (T7 open; Pelaez et al. 2000 dynamic-instability finding, never claimed solved)
    double deploy_failure_prob = 0.05;  // PLACEHOLDER deployment-failure probability [-] (reported only, not applied to years_* below)
};

// Deorbit-time band and diagnostics returned by edt_deorbit_years(). Report
// the [years_optimistic, years_conservative] pair together -- never
// years_optimistic alone (spec:247-249).
struct EdtResult {
    double years_optimistic;    // deorbit time [yr], eta_hi = |cos i| edge (fixed / power-capped current)
    double years_conservative;  // deorbit time [yr], eta_lo = cos(i)^2 edge (EMF / collection-limited current)
    double eta_hi;               // |cos i|: optimistic orbit-averaged inclination efficiency [-]
    double eta_lo;               // cos(i)^2: conservative orbit-averaged inclination efficiency [-]
    double emf_v;                 // open-circuit motional EMF at the initial altitude [V]
    double power_w;               // emf_v * avg_current_a, diagnostic only [W]
};

// General EDT quasi-circular spiral integrator: an along-track Lorentz drag
// force F(a) [N] (any function of semi-major axis a, not necessarily a power
// law) drives, for a circular orbit (Gauss planetary equation),
//   da/dt = -2 * F(a) * a^1.5 / (m * sqrt(mu))
// Integrates from a0_m down to a_stop_m and returns elapsed time [s]. Same
// trapezoidal-in-a quadrature style as integrate_decay_seconds above. For the
// aligned-dipole force law F(a) = C / a^3 this cross-checks the elementary
// closed form used internally by edt_deorbit_years (WP13 task-3 requirement).
template <typename ForceFn>
double integrate_edt_seconds(double a0_m, double a_stop_m, double m,
                             ForceFn force_at_a, int steps = 6000) {
    if (a0_m <= a_stop_m || steps < 1 || m <= 0.0) {
        return 0.0;
    }
    const double da = (a0_m - a_stop_m) / steps;
    const auto integrand = [&](double a) {
        return m * std::sqrt(kEarthMu) / (2.0 * force_at_a(a) * std::pow(a, 1.5));
    };
    double sum = 0.5 * (integrand(a_stop_m) + integrand(a0_m));
    for (int i = 1; i < steps; ++i) {
        sum += integrand(a_stop_m + static_cast<double>(i) * da);
    }
    return sum * da;
}

// Aligned-dipole EDT deorbit-time band for a debris target fitted with an EDT
// kit (see EdtConfig). Orbit-normal field magnitude B_n(a) = kB0 *
// (kDipoleRefRadius_m / a)^3 * |cos i| (SPENVIS centred dipole, IGRF epoch
// 2000; i = target's catalog inclination -- this is the MANDATORY
// inclination-dependent v x B efficiency, spec:240-243). |cos i|, not signed
// cos i: the passive tether's induced-current direction follows the
// motional-EMF sign (Lenz), so the along-track Lorentz force is always a
// drag; a retrograde orbit (i > 90 deg, e.g. the class-C sun-synchronous
// candidate) reverses the EMF polarity but keeps a finite deorbit drag
// scaling with |cos i|. Along-track force at semi-major axis a: F(a) =
// avg_current_a * tether_length_m * B_n(a) * eta_libration is the optimistic
// edge (eta_hi = |cos i|, fixed/power-capped current); the conservative edge
// is F(a) * |cos i| (eta_lo = cos(i)^2, EMF/collection-limited current) --
// see wp13-edt-derivation.md Section 5 for both current-limit regimes.
// Solved in closed form (elementary power-law integral, exact);
// integrate_edt_seconds above cross-checks the algebra. Exactly-polar orbits
// (|cos i| <= 1e-6): the aligned-dipole orbit-normal field is zero
// everywhere on the orbit (derivation note Section 6, limit L3) --
// years_optimistic/years_conservative are +infinity, a physically correct
// null result for this idealization (not a numerical artifact); emitters
// should print "n/a (polar: aligned-dipole avg force -> 0)" for it.
// Libration (T7) is folded in only as the flat eta_libration duty-cycle knob
// above -- dynamic stability itself is explicitly unresolved (never claimed
// solved).
EdtResult edt_deorbit_years(const DebrisCatalog& target, const EdtConfig& cfg,
                            double stop_altitude_km);

}  // namespace adsc
