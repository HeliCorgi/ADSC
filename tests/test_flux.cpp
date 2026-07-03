// T6 flux_sweep tests: pin the hypervelocity-impact and small-debris-flux
// numbers to +/-1% so the committed table cannot drift (R6), and check the
// output carries no wall-clock timestamp (spec v4.2). Explicit return-1 (R4).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "adsc/flux.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

// |actual - expected| <= 1% of |expected|.
static bool near1pct(double actual, double expected) {
    return std::abs(actual - expected) <= 0.01 * std::abs(expected);
}

static std::string slurp(const std::string& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

int main() {
    const FluxConfig cfg;
    const FluxReport r = compute_flux_report(cfg);

    std::printf("flux: %.1f MJ/kg, %.2fx TNT; 1cm Al %.3f g, %.2f kJ, %.2f g TNT; "
                "area %.2f / %.2f km^2\n",
                r.specific_ke_mj_per_kg, r.tnt_ratio, r.sphere_mass_g,
                r.sphere_ke_kj, r.sphere_tnt_g, r.area_km2_avg, r.area_km2_peak);

    // 1. Hypervelocity energy pins (+/-1%).
    CHECK(near1pct(r.specific_ke_mj_per_kg, 50.0));   // 1/2 v^2 = 50 MJ/kg
    CHECK(near1pct(r.tnt_ratio, 11.95));              // ~12x TNT
    CHECK(near1pct(r.sphere_mass_g, 1.4137));         // 1 cm Al sphere
    CHECK(near1pct(r.sphere_ke_kj, 70.69));           // ~71 kJ
    CHECK(near1pct(r.sphere_tnt_g, 16.89));           // ~17 g TNT

    // 2. Collector exposure + area for 1%/yr removal (+/-1%).
    CHECK(near1pct(r.ref_hits_per_yr_avg, 0.037869));
    CHECK(near1pct(r.ref_hits_per_yr_peak, 0.315576));
    CHECK(near1pct(r.ref_interval_yr_avg, 26.41));
    CHECK(near1pct(r.ref_interval_yr_peak, 3.169));
    CHECK(near1pct(r.area_km2_avg, 31.69));           // ~32 km^2 (average density)
    CHECK(near1pct(r.area_km2_peak, 3.803));          // ~3.8 km^2 (peak density)

    // 3. Physical consistency: peak density is ~8.3x average, so area and
    //    interval scale inversely with density.
    CHECK(near1pct(r.area_km2_avg / r.area_km2_peak,
                   cfg.density_peak_per_km3 / cfg.density_avg_per_km3));
    CHECK(near1pct(r.ref_hits_per_yr_peak / r.ref_hits_per_yr_avg,
                   cfg.density_peak_per_km3 / cfg.density_avg_per_km3));

    // 4. Pure-helper sanity: sphere volume/energy formulas.
    CHECK(near1pct(specific_kinetic_energy(10000.0), 5.0e7));
    CHECK(near1pct(kinetic_energy(sphere_mass(0.01, 2700.0), 10000.0), 70686.0));

    // 5. Emitted Markdown carries the numbers and NO timestamp (spec v4.2).
    {
        const std::string p = "test_t6_flux_sweep.md";
        write_flux_md(p, cfg, r);
        const std::string md = slurp(p);
        CHECK(md.find("MJ/kg") != std::string::npos);
        CHECK(md.find("km^2") != std::string::npos);
        // No wall-clock date/time stamps: reject an ISO date and clock patterns.
        CHECK(md.find("UTC") == std::string::npos);
        CHECK(md.find("2026-") == std::string::npos);
        CHECK(md.find("generated at") == std::string::npos);
        CHECK(md.find("GMT") == std::string::npos);
    }

    std::printf("flux: all tests passed\n");
    return 0;
}
