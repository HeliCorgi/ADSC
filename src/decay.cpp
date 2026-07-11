#include "adsc/decay.hpp"

#include <cmath>

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
    return {"CZ upper stage (PLACEHOLDER)", 0.0, 0.0, 0.0, true};
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

}  // namespace adsc
