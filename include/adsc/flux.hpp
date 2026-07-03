#pragma once

#include <string>

namespace adsc {

// ============================================================================
// Small-debris flux / hypervelocity-impact sweep (T6 deliverable)
// ----------------------------------------------------------------------------
// Regenerates the physics table behind the T6 open trade: direct removal of
// 1-10 cm fragments is OUT of scope, and the exclusion is evidence, not
// neglect. The table quantifies (a) the specific kinetic energy of a ~10 km/s
// hypervelocity impactor (~50 MJ/kg, ~12x TNT) and the energy of a 1 cm
// aluminium fragment (~71 kJ, ~17 g TNT), (b) the cm-class hit rate on a
// collector of given area at two spatial densities (LEO average vs a peak
// band), and (c) the collector area required to remove 1 %/yr of the >=1 cm
// population (km^2 scale). The evidence pack's limitations section cites it.
//
// Everything is SI internally (m, kg, s, J). Debris spatial densities are given
// per km^3 and areas reported in km^2 because that is how the literature quotes
// them. All debris-population figures are PLACEHOLDER, marked below and to be
// filled with citations (MASTER-8 / ESA spatial-density class figures) at fill
// time (R10). No wall-clock timestamp is emitted (R6, spec v4.2).
// ============================================================================

// PLACEHOLDER-marked flux parameters (R10); the physical constants (aluminium
// density, TNT specific energy) are standard values, not placeholders.
struct FluxConfig {
    double impact_speed_m_s = 10000.0;   // representative HVI closing speed [m/s]
    double al_density_kg_m3 = 2700.0;    // aluminium density [kg/m^3]
    double sphere_diameter_m = 0.01;     // 1 cm reference fragment [m]
    double tnt_specific_energy_j_per_kg = 4.184e6;  // TNT (1 g TNT = 4184 J)

    // Spatial number density of >=1 cm debris [objects / km^3]. PLACEHOLDER --
    // cite MASTER-8 / ESA spatial-density figures at fill time.
    double density_avg_per_km3  = 1.2e-6;  // PLACEHOLDER LEO-average
    double density_peak_per_km3 = 1.0e-5;  // PLACEHOLDER peak congested band

    // >=1 cm population and the removal-fraction target. PLACEHOLDER (MASTER-8
    // class); cite at fill time.
    double population_ge_1cm       = 1.2e6;  // PLACEHOLDER object count
    double removal_fraction_per_yr = 0.01;   // 1 %/yr

    double reference_collector_m2 = 100.0;   // collector area for the hit-rate demo [m^2]
    double seconds_per_year = 365.25 * 86400.0;
};

// --- pure physics helpers (SI) ---
// Specific kinetic energy 1/2 v^2 [J/kg].
double specific_kinetic_energy(double speed_m_s);
// Mass of a solid sphere [kg]: rho * (pi/6) d^3.
double sphere_mass(double diameter_m, double density_kg_m3);
// Kinetic energy 1/2 m v^2 [J].
double kinetic_energy(double mass_kg, double speed_m_s);
// TNT-equivalent mass [kg] for an energy [J].
double tnt_equivalent_kg(double energy_j, const FluxConfig& cfg);
// cm-class impacts per year on `area_m2` at spatial density `density_per_km3`:
// rate = n * v * A (with v, A converted to km/s, km^2), times seconds/year.
double hit_rate_per_year(double density_per_km3, double area_m2, const FluxConfig& cfg);
// Collector area [km^2] to remove `removal_fraction_per_yr` of `population`
// per year at spatial density `density_per_km3`.
double area_for_removal_km2(double density_per_km3, const FluxConfig& cfg);

// --- report ---
struct FluxReport {
    double specific_ke_mj_per_kg = 0.0;   // ~50
    double tnt_ratio = 0.0;               // specific KE / TNT specific energy (~12)
    double sphere_mass_g = 0.0;           // ~1.41
    double sphere_ke_kj = 0.0;            // ~71
    double sphere_tnt_g = 0.0;            // ~17
    // per density (average, peak):
    double ref_hits_per_yr_avg = 0.0, ref_hits_per_yr_peak = 0.0;
    double ref_interval_yr_avg = 0.0, ref_interval_yr_peak = 0.0;
    double area_km2_avg = 0.0, area_km2_peak = 0.0;   // ~32, ~3.8
};

FluxReport compute_flux_report(const FluxConfig& cfg);

// Write the Markdown table (no timestamp, R6). Same numbers go to stdout in the
// driver.
void write_flux_md(const std::string& path, const FluxConfig& cfg,
                   const FluxReport& r);

}  // namespace adsc
