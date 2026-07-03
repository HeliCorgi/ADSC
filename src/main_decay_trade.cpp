// WP3 decay-trade CSV emitter: writes generated/wp3_decay_trade.csv from the
// EXISTING decay model (src/decay.cpp) so the WP7a visualization has a stable,
// committed data source (R6). This changes no physics and no existing number --
// it re-emits the same sail-area x solar-activity -> decay-years trade the
// adsc_sim scenario-6 demo already prints, as machine-readable CSV. Deterministic
// and timestamp-free (spec v4.2 R6).
//
//   decay_trade [out_dir]
//
// Default out_dir = "generated".
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "adsc/decay.hpp"
#include "adsc/mission.hpp"  // Config

using namespace adsc;

namespace {
// Stable schema id for wp3_decay_trade.csv.
const char* kSchema = "1.0";

// Sail areas swept for the trade curve [m^2]. Spans B's ~7 m^2 and A's
// ~135-2155 m^2 25-year crossings so both are visible.
const double kAreas[] = {5.0, 10.0, 25.0, 50.0, 100.0, 200.0,
                         400.0, 800.0, 1600.0, 3200.0};
}  // namespace

int main(int argc, char** argv) {
    const std::string out_dir = (argc > 1) ? argv[1] : "generated";
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);

    const Config cfg;
    const DebrisCatalog cats[] = {catalog_A(), catalog_B()};
    const double stop_km = cfg.reentry_handoff_altitude_km;

    std::FILE* f = std::fopen((out_dir + "/wp3_decay_trade.csv").c_str(), "w");
    if (!f) return 1;
    std::fprintf(f,
        "schema_version,catalog,mass_kg,altitude_km,record_type,sail_area_m2,"
        "value_solar_max,value_solar_min,units,notes\n");

    for (const DebrisCatalog& c : cats) {
        // Decay years vs sail area (band = solar max .. solar min).
        for (double area : kAreas) {
            const double y_max = sail_decay_years(c, area, cfg.kit_mass_kg,
                cfg.drag_cd, stop_km, cfg.solar_max_density_factor);
            const double y_min = sail_decay_years(c, area, cfg.kit_mass_kg,
                cfg.drag_cd, stop_km, cfg.solar_min_density_factor);
            std::fprintf(f, "%s,%s,%.1f,%.1f,decay_years,%.4f,%.4f,%.4f,years,\"\"\n",
                         kSchema, c.name, c.mass_kg, c.altitude_km, area, y_max, y_min);
        }
        // Sail area required to meet the 25-year guideline (solar max .. min).
        const double a_max = area_for_target_years(c, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_max_density_factor);
        const double a_min = area_for_target_years(c, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_min_density_factor);
        std::fprintf(f, "%s,%s,%.1f,%.1f,area_for_25yr,0.0,%.4f,%.4f,m2,"
                        "\"sail area for the 25-yr guideline (solar max .. min)\"\n",
                     kSchema, c.name, c.mass_kg, c.altitude_km, a_max, a_min);
    }
    // The 25-year guideline reference line.
    std::fprintf(f, "%s,,0.0,0.0,guideline,0.0,25.0,25.0,years,"
                    "\"IADC 25-year post-mission disposal guideline\"\n", kSchema);
    std::fclose(f);

    std::printf("[WP3] wrote %s/wp3_decay_trade.csv (%zu areas x %zu catalogs, "
                "stop %.0f km)\n", out_dir.c_str(),
                sizeof(kAreas) / sizeof(kAreas[0]),
                sizeof(cats) / sizeof(cats[0]), stop_km);
    return 0;
}
