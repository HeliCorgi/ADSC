#pragma once

#include <string>
#include <vector>

#include "adsc/campaign.hpp"  // CampaignConfig, RunResult, run_campaign
#include "adsc/decay.hpp"     // DebrisCatalog
#include "adsc/mission.hpp"   // Config

namespace adsc {

// ============================================================================
// Parametric cost model + figure of merit (WP6)
// ----------------------------------------------------------------------------
// Consumes the WP5 campaign (schema 1.0): the amortization sweep re-runs the
// WP5 engine (run_campaign) across a kit-inventory sweep at the fixed master
// seed, and the baseline point (n_kits == CampaignConfig::kits_initial)
// reproduces generated/wp5_campaign_runs.csv exactly. Emits its own stable
// schema (schema_version below) for WP7 to consume.
//
// Cost model (spec §5 WP6):
//   C_campaign = C_dev + C_bus(m_dry) + N*C_kit + C_launch(m_wet, band)
//                + C_ops(T)
// The PRIMARY output is in RELATIVE cost units (CU). Absolute currency is NEVER
// emitted as a point value (R6/D10): only a cited RANGE via the PLACEHOLDER
// CU->currency anchor, left unfilled here (filled with citations in WP7).
//
// Figure of merit (spec §4): FoM = sum_i m_i*w(h_i) / C_campaign, a
// debris-risk-reduction-per-cost (cascade-fuel) metric. Reported under >=2
// normalized congestion weightings; the ranking disagreement is open trade T5.
//
// Distribution propagation: cost, cost/removal and FoM are computed PER RUN
// from the WP5 per-run removals/mission-time and reported as p05/p50/p95 --
// never mean-only.
//
// All parameters live in CostConfig and are marked PLACEHOLDER (R10). WP6
// implements the cost model / amortization / tornado / FoM only -- no WP7
// evidence pack, no charts.
// ============================================================================

inline const char* wp6_schema_version() { return "1.0"; }

// A congestion-weight anchor: normalized (peak ~1) weight of a target band at
// `altitude_km` under each weighting. PLACEHOLDER values -- fill with citations
// (spatial: public LEO spatial-density data, e.g. ESA MASTER; criticality: a
// published criticality-style index, e.g. an ESA environmental-index / Rossi
// class ranking) in WP7. The two columns deliberately peak at different
// altitudes (spatial ~750 km, criticality ~840 km), so the band ranking flips
// between weightings -- that disagreement is open trade T5.
struct WeightAnchor {
    double altitude_km;
    double spatial;      // PLACEHOLDER normalized spatial-density weight
    double criticality;  // PLACEHOLDER normalized criticality-style weight
};

enum class Weighting { SpatialDensity, Criticality };
const char* weighting_label(Weighting w);

// All PLACEHOLDER; grouped so nothing is a bare literal in the cost logic (R10).
struct CostConfig {
    // --- relative cost-unit (CU) coefficients ---
    double c_dev_cu           = 100.0;  // PLACEHOLDER program development (per-campaign allocation) [CU]
    double c_bus_cu           = 3.0;    // PLACEHOLDER bus mass-CER coefficient [CU]
    double c_bus_exponent     = 0.7;    // PLACEHOLDER bus mass-CER exponent [-]
    double c_kit_cu           = 4.0;    // PLACEHOLDER per-kit cost [CU]
    double c_launch_cu_per_kg = 0.5;    // PLACEHOLDER launch cost coefficient [CU/kg]
    double c_ops_cu_per_day   = 2.0;    // PLACEHOLDER operations cost [CU/day]

    // Launch band factor (higher / more-inclined orbits cost more). PLACEHOLDER.
    double launch_band_ref_km        = 700.0;  // PLACEHOLDER reference altitude [km]
    double launch_band_per_100km     = 0.06;   // PLACEHOLDER cost slope [-/100 km]
    double launch_band_ref_incl_deg  = 60.0;   // PLACEHOLDER reference inclination [deg]
    double launch_band_per_deg       = 0.004;  // PLACEHOLDER cost slope [-/deg]

    // Kit-inventory amortization sweep bounds (inclusive).
    int sweep_kit_min = 1;
    int sweep_kit_max = 8;

    // Tornado sensitivity half-width (fractional one-at-a-time perturbation).
    double tornado_delta_frac = 0.30;   // PLACEHOLDER +/-30%

    // CU -> currency anchor RANGE. Left 0 => NOT filled; no dollar value is
    // emitted. Fill with a CITED range (public launch $/kg + smallsat bus cost
    // ranges) in WP7; a point-value dollar claim is forbidden (R6/D10).
    double cu_to_musd_low  = 0.0;  // PLACEHOLDER (cited range, WP7)
    double cu_to_musd_high = 0.0;  // PLACEHOLDER (cited range, WP7)

    // Normalized congestion-weight table (PLACEHOLDER; cite on fill). Peaks
    // differ by column so band ranking flips between weightings (T5).
    std::vector<WeightAnchor> weight_table = {
        {600.0, 0.35, 0.30}, {700.0, 0.75, 0.55}, {750.0, 1.00, 0.68},
        {800.0, 0.92, 0.85}, {840.0, 0.78, 1.00}, {900.0, 0.55, 0.90},
        {1000.0, 0.30, 0.60},
    };
};

// Piecewise-linear normalized congestion weight at `altitude_km` (clamped to
// the table ends).
double congestion_weight(Weighting w, double altitude_km, const CostConfig& cc);

// Launch band factor for a target band (PLACEHOLDER model).
double launch_band_factor(const CostConfig& cc, double altitude_km,
                          double inclination_deg);

// Total campaign cost [CU] for one mission that CARRIES n_kits and lasts
// mission_time_s. (Cost counts kits carried/launched, not just installed --
// carrying more kits than the Δv budget can install is what makes the
// amortization curve turn back up.)
double mission_campaign_cost(const CostConfig& cc, const Config& base,
                             const DebrisCatalog& cat, int n_kits,
                             double mission_time_s);

// --- structured results (both CSV and MD writers consume these) ---
struct AmortPoint {
    int    n_kits = 0;
    double cost_per_removal_p05 = 0.0, cost_per_removal_p50 = 0.0, cost_per_removal_p95 = 0.0; // CU/removal
    double ratio_to_n1_p50 = 0.0;    // cost/removal(N).p50 / cost/removal(min).p50
    double removals_p50 = 0.0;
    double campaign_cost_p50 = 0.0;  // CU
    long   zero_removal_runs = 0;    // excluded from cost/removal
};

struct FoMResult {
    Weighting weighting = Weighting::SpatialDensity;
    double band_weight = 0.0;         // w(h) at the catalog altitude
    double removed_mass_kg_p50 = 0.0; // per mission
    double fom_p05 = 0.0, fom_p50 = 0.0, fom_p95 = 0.0;  // kg/CU
};

struct TornadoRow {
    std::string param;
    double cost_per_removal_low = 0.0, cost_per_removal_high = 0.0, cost_per_removal_swing = 0.0;
    double fom_low = 0.0, fom_high = 0.0, fom_swing = 0.0;  // spatial FoM
};

struct CostComponent { std::string name; double cu = 0.0; };  // baseline breakdown

struct CatalogCost {
    DebrisCatalog            catalog;
    int                      baseline_n_kits = 0;
    std::vector<AmortPoint>  amortization;   // one per swept N
    std::vector<FoMResult>   fom;            // one per weighting
    std::vector<CostComponent> baseline_components;  // C_dev/bus/kit/launch/ops at baseline
};

struct CostReport {
    std::vector<CatalogCost> catalogs;
    std::vector<TornadoRow>  tornado;        // representative: first catalog, baseline N, spatial FoM
    std::string              tornado_catalog;
    double                   elapsed_s = 0.0;
};

// Compute the whole report (re-runs the WP5 campaign across the kit sweep).
CostReport compute_cost_report(const std::vector<DebrisCatalog>& catalogs,
                               const CampaignConfig& ccfg, const Config& base_cfg,
                               const CostConfig& cc);

// Consume a committed WP5 summary.csv (schema 1.0): return the (catalog,metric)
// -> value map for the requested column ("estimate" or "p50"). Empty on error.
// Used to cross-check the WP6 baseline against the committed WP5 numbers.
double read_wp5_summary_value(const std::string& path, const std::string& catalog,
                              const std::string& metric, const std::string& column);

// --- output IO ---
void write_cost_summary_csv(const std::string& path, const CostConfig& cc,
                            const CostReport& report);
void write_cost_summary_md(const std::string& path, const CostConfig& cc,
                           const CostReport& report);
void write_cost_schema_md(const std::string& path);

}  // namespace adsc
