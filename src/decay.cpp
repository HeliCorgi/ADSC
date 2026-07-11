#include "adsc/decay.hpp"

#include <cmath>
#include <limits>

namespace adsc {

namespace {

struct Band {
    double h0_km;
    double rho0;   // base density [kg/m^3]
    double H_km;   // scale height [km]
};

// Vallado, "Fundamentals of Astrodynamics and Applications" (4th ed.), Table
// 8-4: exponential atmospheric density model (nominal / solar-mean). rho(h) =
// rho0 * exp(-(h - h0)/H) within each band.
constexpr Band kBands[] = {
    {0.0,    1.225,     7.249},   {25.0,   3.899e-2,  6.349},
    {30.0,   1.774e-2,  6.682},   {40.0,   3.972e-3,  7.554},
    {50.0,   1.057e-3,  8.382},   {60.0,   3.206e-4,  7.714},
    {70.0,   8.770e-5,  6.549},   {80.0,   1.905e-5,  5.799},
    {90.0,   3.396e-6,  5.382},   {100.0,  5.297e-7,  5.877},
    {110.0,  9.661e-8,  7.263},   {120.0,  2.438e-8,  9.473},
    {130.0,  8.484e-9,  12.636},  {140.0,  3.845e-9,  16.149},
    {150.0,  2.070e-9,  22.523},  {180.0,  5.464e-10, 29.740},
    {200.0,  2.789e-10, 37.105},  {250.0,  7.248e-11, 45.546},
    {300.0,  2.418e-11, 53.628},  {350.0,  9.518e-12, 53.298},
    {400.0,  3.725e-12, 58.515},  {450.0,  1.585e-12, 60.828},
    {500.0,  6.967e-13, 63.822},  {600.0,  1.454e-13, 71.835},
    {700.0,  3.614e-14, 88.667},  {800.0,  1.170e-14, 124.64},
    {900.0,  5.245e-15, 181.05},  {1000.0, 3.019e-15, 268.00},
};
constexpr int kNumBands = static_cast<int>(sizeof(kBands) / sizeof(kBands[0]));
constexpr double kSecondsPerYear = 365.25 * 86400.0;

// SPENVIS (ESA/BIRA-IASB Space Environment Information System), "Dipole
// approximations of the geomagnetic field", centred-dipole IGRF epoch-2000
// values: equatorial surface field B0 and the reference (mean) Earth radius
// used in the dipole formula (https://www.spenvis.oma.be/help/background/
// magfield/cd.html, web-verified 2026-07-11, wp13-literature.md Topic 4).
// Deliberately NOT kEarthRadius (WGS-84 equatorial, relmotion.hpp): the cited
// dipole formula is defined against this specific mean-radius reference.
constexpr double kB0 = 3.01153e-5;               // T
constexpr double kDipoleRefRadius_m = 6371200.0;  // m

}  // namespace

double atmospheric_density(double altitude_m, double solar_factor) {
    double h_km = altitude_m / 1000.0;
    if (h_km < 0.0) h_km = 0.0;

    int b = 0;
    for (int i = 0; i < kNumBands; ++i) {
        if (h_km >= kBands[i].h0_km) {
            b = i;
        } else {
            break;
        }
    }
    const Band& band = kBands[b];
    const double rho = band.rho0 * std::exp(-(h_km - band.h0_km) / band.H_km);
    return rho * solar_factor;
}

DebrisCatalog catalog_A() {
    return {"SL-16 / Zenit-2 second stage", 9000.0, 840.0, 71.0, false};
}
DebrisCatalog catalog_B() {
    return {"SL-8 / Kosmos-3M second stage", 1400.0, 750.0, 78.0, false};
}
DebrisCatalog catalog_C() {
    // Envisat (ESA): mass 8211 kg (Service Module 2673 + PEB 1021 + Payload
    // Carrier 2078 + fuel 319 + instruments 2118 kg); sun-synchronous orbit
    // ~765-800 km, inclination ~98.4-98.55 deg; defunct since April 2012 --
    // the archetypal massive / controlled-reentry-class Class-C target.
    // eoPortal (https://www.eoportal.org/satellite-missions/envisat),
    // web-verified 2026-07-11 (wp13-literature.md Topic 6, item 1).
    return {"Envisat-class massive SSO payload", 8211.0, 780.0, 98.4, false};
}
DebrisCatalog catalog_D() {
    return {"US Delta-class stage (PLACEHOLDER)", 0.0, 0.0, 0.0, true};
}

double sail_decay_years(const DebrisCatalog& target, double sail_area_m2,
                        double kit_mass_kg, double cd, double stop_altitude_km,
                        double solar_factor) {
    const double m_total = target.mass_kg + kit_mass_kg;
    if (m_total <= 0.0) return 0.0;
    const double area_over_mass = sail_area_m2 / m_total;
    const double a0     = kEarthRadius + target.altitude_km * 1000.0;
    const double a_stop = kEarthRadius + stop_altitude_km * 1000.0;
    const double t = integrate_decay_seconds(
        a0, a_stop, cd, area_over_mass,
        [solar_factor](double alt_m) { return atmospheric_density(alt_m, solar_factor); });
    return t / kSecondsPerYear;
}

double area_for_target_years(const DebrisCatalog& target, double target_years,
                             double kit_mass_kg, double cd, double stop_altitude_km,
                             double solar_factor) {
    if (target_years <= 0.0) return 0.0;
    // Decay time is exactly inversely proportional to A/m (and hence to sail
    // area at fixed mass), so evaluate once at 1 m^2 and scale.
    const double years_at_1m2 =
        sail_decay_years(target, 1.0, kit_mass_kg, cd, stop_altitude_km, solar_factor);
    return years_at_1m2 / target_years;
}

double edt_deorbit_days(double a0_m, double a_stop_m, double delta_a_per_day_m) {
    if (delta_a_per_day_m <= 0.0 || a0_m <= a_stop_m) return 0.0;
    return (a0_m - a_stop_m) / delta_a_per_day_m;
}

EdtResult edt_deorbit_years(const DebrisCatalog& target, const EdtConfig& cfg,
                            double stop_altitude_km) {
    EdtResult result{};

    // Magnitude |cos i|: the passive tether's induced-current direction is set
    // by the motional-EMF sign (Lenz), so the along-track Lorentz force is
    // ALWAYS a drag. For a retrograde orbit (i > 90 deg, e.g. the class-C
    // sun-synchronous candidate) the EMF polarity reverses but the deorbit
    // force magnitude scales with |cos i|, not signed cos i.
    const double cos_i = std::cos(target.inclination_deg * kPi / 180.0);
    const double abs_cos_i = std::fabs(cos_i);
    result.eta_hi = abs_cos_i;
    result.eta_lo = cos_i * cos_i;

    const double a0 = kEarthRadius + target.altitude_km * 1000.0;
    const double Bn0 = kB0 * std::pow(kDipoleRefRadius_m / a0, 3.0) * abs_cos_i;
    const double v0 = std::sqrt(kEarthMu / a0);
    // EMF/power reported as magnitudes; polarity reverses for retrograde
    // orbits (diagnostic only -- the drag sign is unaffected, see above).
    result.emf_v = v0 * Bn0 * cfg.tether_length_m;
    result.power_w = result.emf_v * cfg.avg_current_a;

    if (abs_cos_i <= 1e-6) {
        // Exactly-polar orbit: the aligned-dipole orbit-normal field is zero
        // everywhere on the orbit (wp13-edt-derivation.md Section 6, limit
        // L3), so the along-track EDT force -> 0 identically, not just on
        // average. This is the correct physics for the aligned-dipole
        // idealization, not a numerical artifact. (Retrograde orbits with
        // |cos i| > 0, e.g. i = 98.4 deg, are handled normally above.)
        result.years_optimistic = std::numeric_limits<double>::infinity();
        result.years_conservative = std::numeric_limits<double>::infinity();
        return result;
    }

    const double a_stop = kEarthRadius + stop_altitude_km * 1000.0;
    const double m = target.mass_kg + cfg.kit_mass_kg;
    if (a0 <= a_stop || m <= 0.0) {
        result.years_optimistic = 0.0;
        result.years_conservative = 0.0;
        return result;
    }

    // Optimistic edge: F_hi(a) = I * L * B_n(a) * eta_lib
    //                          = (I * L * kB0 * kDipoleRefRadius_m^3 * |cos i| * eta_lib) / a^3
    //                          = C_hi / a^3.
    // Conservative edge: F_lo(a) = F_hi(a) * |cos i| = C_lo / a^3
    // (wp13-edt-derivation.md Section 5: the two current-limit regimes).
    const double C_hi = cfg.avg_current_a * cfg.tether_length_m * kB0 *
                        std::pow(kDipoleRefRadius_m, 3.0) * abs_cos_i * cfg.eta_libration;
    const double C_lo = C_hi * abs_cos_i;

    // Quasi-circular spiral under F(a) = C / a^3:
    //   da/dt = -2*F(a)*a^1.5/(m*sqrt(mu)) = -(2*C/(m*sqrt(mu))) * a^-1.5
    //   => a^1.5 da = -(2*C/(m*sqrt(mu))) dt
    //   => (2/5)(a_stop^2.5 - a0^2.5) = -(2*C/(m*sqrt(mu))) * t
    //   => t = m*sqrt(mu)*(a0^2.5 - a_stop^2.5) / (5*C)
    const double sqrt_mu = std::sqrt(kEarthMu);
    const double delta_a25 = std::pow(a0, 2.5) - std::pow(a_stop, 2.5);
    result.years_optimistic = (m * sqrt_mu * delta_a25 / (5.0 * C_hi)) / kSecondsPerYear;
    result.years_conservative = (m * sqrt_mu * delta_a25 / (5.0 * C_lo)) / kSecondsPerYear;
    return result;
}

}  // namespace adsc
