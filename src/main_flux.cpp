// T6 flux_sweep: regenerates the small-debris / hypervelocity-impact table to
// stdout and generated/t6_flux_sweep.md (R6/R12, spec v4.2 — no timestamp).
//
//   flux_sweep [out_dir]
//
// Default out_dir = "generated".
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>

#include "adsc/flux.hpp"

using namespace adsc;

int main(int argc, char** argv) {
    const std::string out_dir = (argc > 1) ? argv[1] : "generated";
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);

    const FluxConfig cfg;
    const FluxReport r = compute_flux_report(cfg);

    std::printf("=== ADSC T6 — small-debris flux / hypervelocity-impact sweep ===\n");
    std::printf("(fragment removal OUT of scope; documented physics, not neglect)\n\n");

    std::printf("[T6] hypervelocity impact @ %.0f km/s\n", cfg.impact_speed_m_s / 1000.0);
    std::printf("  specific kinetic energy : %.1f MJ/kg (%.1fx TNT specific energy)\n",
                r.specific_ke_mj_per_kg, r.tnt_ratio);
    std::printf("  1 cm Al sphere          : %.2f g, %.1f kJ (= %.1f g TNT)\n",
                r.sphere_mass_g, r.sphere_ke_kj, r.sphere_tnt_g);

    std::printf("\n[T6] collector exposure (>= 1 cm; PLACEHOLDER densities, cite MASTER-8)\n");
    std::printf("  reference collector     : %.0f m^2\n", cfg.reference_collector_m2);
    std::printf("  average (%.1e /km^3)   : %.4f hits/yr (1 per %.1f yr), "
                "area for %.0f%%/yr = %.1f km^2\n",
                cfg.density_avg_per_km3, r.ref_hits_per_yr_avg, r.ref_interval_yr_avg,
                cfg.removal_fraction_per_yr * 100.0, r.area_km2_avg);
    std::printf("  peak    (%.1e /km^3)   : %.4f hits/yr (1 per %.1f yr), "
                "area for %.0f%%/yr = %.1f km^2\n",
                cfg.density_peak_per_km3, r.ref_hits_per_yr_peak, r.ref_interval_yr_peak,
                cfg.removal_fraction_per_yr * 100.0, r.area_km2_peak);

    write_flux_md(out_dir + "/t6_flux_sweep.md", cfg, r);
    std::printf("\nwrote %s/t6_flux_sweep.md\n", out_dir.c_str());
    std::printf("=== T6 flux sweep complete ===\n");
    return 0;
}
