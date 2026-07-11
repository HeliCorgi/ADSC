#include "adsc/flux.hpp"

#include <cmath>
#include <cstdio>

namespace adsc {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double specific_kinetic_energy(double speed_m_s) {
    return 0.5 * speed_m_s * speed_m_s;
}

double sphere_mass(double diameter_m, double density_kg_m3) {
    // Volume of a sphere = (pi/6) d^3.
    return density_kg_m3 * (kPi / 6.0) * diameter_m * diameter_m * diameter_m;
}

double kinetic_energy(double mass_kg, double speed_m_s) {
    return 0.5 * mass_kg * speed_m_s * speed_m_s;
}

double tnt_equivalent_kg(double energy_j, const FluxConfig& cfg) {
    return energy_j / cfg.tnt_specific_energy_j_per_kg;
}

double hit_rate_per_year(double density_per_km3, double area_m2,
                         const FluxConfig& cfg) {
    // rate[1/s] = n[1/km^3] * v[km/s] * A[km^2]; convert v and A from SI.
    const double v_km_s  = cfg.impact_speed_m_s / 1000.0;
    const double area_km2 = area_m2 / 1.0e6;
    return density_per_km3 * v_km_s * area_km2 * cfg.seconds_per_year;
}

double area_for_removal_km2(double density_per_km3, const FluxConfig& cfg) {
    // Removing f*pop objects/yr at flux (n * v * seconds/yr) per km^2 needs
    // A = (f*pop) / (n * v * seconds/yr).
    const double v_km_s = cfg.impact_speed_m_s / 1000.0;
    const double flux_per_km2_yr = density_per_km3 * v_km_s * cfg.seconds_per_year;
    if (flux_per_km2_yr <= 0.0) return 0.0;
    return (cfg.removal_fraction_per_yr * cfg.population_ge_1cm) / flux_per_km2_yr;
}

FluxReport compute_flux_report(const FluxConfig& cfg) {
    FluxReport r;
    const double spec_ke = specific_kinetic_energy(cfg.impact_speed_m_s);
    r.specific_ke_mj_per_kg = spec_ke / 1.0e6;
    r.tnt_ratio = spec_ke / cfg.tnt_specific_energy_j_per_kg;

    const double m = sphere_mass(cfg.sphere_diameter_m, cfg.al_density_kg_m3);
    const double ke = kinetic_energy(m, cfg.impact_speed_m_s);
    r.sphere_mass_g = m * 1000.0;
    r.sphere_ke_kj = ke / 1000.0;
    r.sphere_tnt_g = tnt_equivalent_kg(ke, cfg) * 1000.0;

    r.ref_hits_per_yr_avg =
        hit_rate_per_year(cfg.density_avg_per_km3, cfg.reference_collector_m2, cfg);
    r.ref_hits_per_yr_peak =
        hit_rate_per_year(cfg.density_peak_per_km3, cfg.reference_collector_m2, cfg);
    r.ref_interval_yr_avg =
        r.ref_hits_per_yr_avg > 0.0 ? 1.0 / r.ref_hits_per_yr_avg : 0.0;
    r.ref_interval_yr_peak =
        r.ref_hits_per_yr_peak > 0.0 ? 1.0 / r.ref_hits_per_yr_peak : 0.0;

    r.area_km2_avg  = area_for_removal_km2(cfg.density_avg_per_km3, cfg);
    r.area_km2_peak = area_for_removal_km2(cfg.density_peak_per_km3, cfg);
    return r;
}

void write_flux_md(const std::string& path, const FluxConfig& cfg,
                   const FluxReport& r) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "# T6 — small-debris flux / hypervelocity-impact sweep\n\n");
    std::fprintf(f,
        "Evidence for the T6 open trade: direct removal of 1-10 cm fragments is "
        "**out of scope**, and this table shows why (it is documented physics, "
        "not neglect). Regenerate with `flux_sweep`. All debris-population "
        "figures are PLACEHOLDER (MASTER-8 / ESA spatial-density class; cite at "
        "fill time). No timestamp is embedded (R6).\n\n");

    std::fprintf(f, "## Hypervelocity impact energy (%.0f km/s)\n\n",
                 cfg.impact_speed_m_s / 1000.0);
    std::fprintf(f, "| quantity | value |\n|---|---:|\n");
    std::fprintf(f, "| specific kinetic energy | %.1f MJ/kg |\n", r.specific_ke_mj_per_kg);
    std::fprintf(f, "| TNT specific-energy ratio | %.1fx |\n", r.tnt_ratio);
    std::fprintf(f, "| 1 cm Al sphere mass | %.2f g |\n", r.sphere_mass_g);
    std::fprintf(f, "| 1 cm Al sphere kinetic energy | %.1f kJ |\n", r.sphere_ke_kj);
    std::fprintf(f, "| 1 cm Al sphere TNT equivalent | %.1f g |\n\n", r.sphere_tnt_g);

    std::fprintf(f, "## Collector exposure (>= 1 cm objects)\n\n");
    std::fprintf(f, "PLACEHOLDER spatial densities: average %.1e /km^3, peak "
                    "%.1e /km^3. Reference collector %.0f m^2.\n\n",
                 cfg.density_avg_per_km3, cfg.density_peak_per_km3,
                 cfg.reference_collector_m2);
    std::fprintf(f, "| density | hits/yr on %.0f m^2 | mean interval | "
                    "area for %.0f%%/yr removal |\n|---|---:|---:|---:|\n",
                 cfg.reference_collector_m2, cfg.removal_fraction_per_yr * 100.0);
    std::fprintf(f, "| average (%.1e /km^3) | %.4f | %.1f yr | %.1f km^2 |\n",
                 cfg.density_avg_per_km3, r.ref_hits_per_yr_avg,
                 r.ref_interval_yr_avg, r.area_km2_avg);
    std::fprintf(f, "| peak (%.1e /km^3) | %.4f | %.1f yr | %.1f km^2 |\n\n",
                 cfg.density_peak_per_km3, r.ref_hits_per_yr_peak,
                 r.ref_interval_yr_peak, r.area_km2_peak);

    std::fprintf(f,
        "**Reading.** A cm-class impactor carries grenade-class energy (~%.0f g "
        "TNT); no material catches it intact -- shields work by shattering and "
        "shedding secondary ejecta. Removing %.0f%%/yr of the >= 1 cm population "
        "needs a **km^2-scale** collector (%.1f km^2 at average density, %.1f "
        "km^2 in a peak band) that is itself the largest collision cross-section "
        "in the band. Hence T6: fragment removal is excluded; laser "
        "photon-pressure/ablation nudging is the research lane (reference only, "
        "no implementation).\n",
        r.sphere_tnt_g, cfg.removal_fraction_per_yr * 100.0, r.area_km2_avg,
        r.area_km2_peak);
    std::fclose(f);
}

}  // namespace adsc
