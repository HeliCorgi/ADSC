#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "adsc/decay.hpp"    // DebrisCatalog
#include "adsc/mission.hpp"  // Config, run_tumble_sync, ActuatorError

namespace adsc {

// ============================================================================
// Campaign Monte Carlo (WP5)
// ----------------------------------------------------------------------------
// One mission processes N class-level catalog targets sequentially in a single
// orbital plane (batch amortization, D2). Between targets, phasing is modeled
// as a parameterized Delta-v / time cost (PLACEHOLDER, no plane-change
// optimization, spec section 7). The mission ends when the Delta-v budget is
// exhausted, kit inventory reaches zero, all targets are processed, or a
// terminal keep-out violation occurs.
//
// Dispersions (all PLACEHOLDER where not physically validated, centralized in
// CampaignConfig) drive per-run variation. The sync leg reuses the pinned,
// truth-driven run_tumble_sync primitive for tractable N>=500 Monte Carlo; the
// dispersions that flow through it are the initial-condition and vehicle ones
// (tumble rate/axis, servicer initial attitude, actuator torque-scale and
// misalignment). Sensor noise/bias dispersions are drawn and their realized
// scale is recorded, but their closed-loop effect is characterized by the WP4
// estimate-driven acceptance rather than re-simulated in every campaign run --
// a documented WP5 simplification (see README known limits).
//
// Determinism (R6): one fixed master seed; every per-run seed is derived from
// it by a SplitMix64 finalizer of (master_seed XOR catalog_salt, run_index),
// where catalog_salt is an FNV-1a hash of the preset name so each catalog runs
// an independent stream. No nondeterministic random_device is ever used for
// committed outputs.
//
// Class-independence (honest scope): the attitude-sync leg and the flat Delta-v
// / kit cost model do not depend on target mass or orbit, so aggregate outcomes
// are largely class-independent by construction; the one orbit-dependent piece
// is the keep-out abort screen, which uses each catalog's altitude. Per-catalog
// stats therefore differ mainly by independent sampling. Class-specific physics
// lives in WP3 decay (already catalog-specific) and future WP6 cost.
//
// SCOPE: WP5 implements the campaign layer and its statistics only. It does not
// implement WP6 cost/FoM, WP7 visualization/evidence, or WP8 compliance
// tooling. The CSV/schema are stabilized so those later WPs can consume them
// without changing the schema. WP5 performs NO legal or regulatory
// determination; the compliance-metadata columns are passive research-profile
// flags, not approvals.
// ============================================================================

// Stable schema identifier stamped into every generated CSV row. Do not change
// casually after this PR: WP6/WP7/WP8 tooling keys off it.
inline const char* wp5_schema_version() { return "1.0"; }

// z for a 95% two-sided interval (spec-fixed constant).
constexpr double kWilsonZ95 = 1.959963984540054;

// ------------------------------------------------------------------ pure stats
// SplitMix64 finalizer: deterministic per-run seed from a master seed and index
// (seed_i = SplitMix64(master_seed, i)). Standard constants; regenerable (R6).
uint64_t splitmix64_seed(uint64_t master_seed, uint64_t index);

// Wilson score interval for a binomial proportion (k successes of n), at z
// sigmas. Handles the k = 0 and k = n edge cases (asymmetric, never [0,0] or
// [1,1] for n > 0). Returns {low, high}, both clamped to [0, 1]. For n = 0 the
// point estimate is undefined and {0, 0} is returned.
struct Interval { double low, high; };
Interval wilson_interval(long k, long n, double z);

// Linear-interpolation percentile (the NumPy / "type 7" convention) of an
// already-sorted ascending sample. p in [0, 100]. Empty sample returns 0.
double percentile_sorted(const std::vector<double>& sorted_ascending, double p);

// -------------------------------------------------------------- campaign model
// Terminal run outcome AND per-target failure classification share these exact
// labels (spec WP5). Run outcomes are {completed, dv_exhausted, kit_exhausted,
// keep_out_violation, other}; gate_abort and sync_timeout are per-target events
// (a safe abort or a failed sync lets the servicer move to the next target,
// so they are not mission-terminal). "other" is a never-expected safety net.
enum class Outcome {
    Completed,
    GateAbort,
    SyncTimeout,
    DvExhausted,
    KitExhausted,
    KeepOutViolation,
    Other
};
const char* outcome_label(Outcome o);

// All PLACEHOLDER values are marked; none is a physically validated figure.
// Grouped so nothing is a bare literal in the campaign logic (R10).
struct CampaignConfig {
    // --- randomness (R6) ---
    uint64_t master_seed = 0x5AD5C0DECAFE2026ULL;  // fixed master seed
    int      n_runs      = 500;                    // full target (>=500, spec)

    // --- campaign layer ---
    int    targets_per_mission = 6;      // N targets in one plane per mission
    int    kits_initial        = 4;      // installed-kit inventory
    double dv_budget_m_s       = 140.0;  // PLACEHOLDER servicer Delta-v budget [m/s]

    // Per-leg Delta-v cost model (PLACEHOLDER; parameterized, no plane-change
    // optimization -- inter-target phasing is a flat cost per hop, spec sec 7).
    double dv_approach_m_s = 8.0;    // PLACEHOLDER rendezvous/approach [m/s]
    double dv_sync_m_s     = 3.0;    // PLACEHOLDER proximity + attitude sync [m/s]
    double dv_depart_m_s   = 5.0;    // PLACEHOLDER safe departure [m/s]
    double dv_abort_m_s    = 4.0;    // PLACEHOLDER safe-abort maneuver [m/s]
    double dv_phasing_m_s  = 15.0;   // PLACEHOLDER inter-target phasing hop [m/s]

    // Per-leg time model (PLACEHOLDER), for the mission elapsed-time metric.
    double t_attach_s   = 300.0;      // PLACEHOLDER clamp + install [s]
    double t_depart_s   = 600.0;      // PLACEHOLDER departure settle [s]
    double t_phasing_s  = 86400.0;    // PLACEHOLDER inter-target phasing [s]

    // --- dispersions (PLACEHOLDER 1-sigma unless noted) ---
    // Closing-speed gate: MUST be wide enough that the capture closing speed
    // crosses the existing Config::max_v_rel (0.15 m/s) abort gate on a
    // non-negligible fraction of runs, so the campaign has real abort exposure.
    double nominal_closing_m_s    = 0.10;   // PLACEHOLDER nominal capture closing speed [m/s]
    double disp_closing_sigma_m_s = 0.045;  // PLACEHOLDER (P(>0.15) ~ 0.13) [m/s]

    // Initial relative state (feeds the dispersed safe-abort keep-out check).
    double disp_rel_pos_m   = 40.0;   // PLACEHOLDER per-axis initial rel-position [m]
    double disp_rel_vel_m_s = 0.02;   // PLACEHOLDER per-axis initial rel-velocity [m/s]

    // Tumble + servicer initial attitude (flow through run_tumble_sync).
    double disp_tumble_rate_frac = 0.40;  // PLACEHOLDER fractional 1-sigma on |w_t| [-]
    double disp_tumble_axis_rad  = 0.30;  // PLACEHOLDER tumble-axis tilt 1-sigma [rad]
    double disp_init_att_rad     = 0.35;  // PLACEHOLDER servicer att-offset 1-sigma [rad]

    // Actuator (flow through run_tumble_sync via ActuatorError).
    double disp_actuator_scale        = 0.10;  // PLACEHOLDER ~+/-10% torque-scale 1-sigma [-]
    double disp_actuator_misalign_rad = 0.01;  // PLACEHOLDER axis-misalignment 1-sigma [rad]

    // Sensor (drawn + recorded; NOT propagated through the truth-driven sync --
    // see the header note and README known limits).
    double disp_sensor_noise_frac = 0.30;   // PLACEHOLDER fractional 1-sigma on sensor sigmas [-]
    double disp_sensor_bias_rad   = 5.0e-4; // PLACEHOLDER sensor bias 1-sigma [rad]

    // Solar activity (drawn + recorded per run for downstream WP3/WP6 decay
    // cost; WP5 does not gate on it).
    double nominal_solar_factor   = 1.0;   // PLACEHOLDER mean atmospheric-density factor [-]
    double disp_solar_factor_frac = 0.50;  // PLACEHOLDER fractional 1-sigma [-]

    // Sync convergence budget per target [s]. Sync is "achieved" if the WP2
    // criteria are declared within this budget; otherwise the target is a
    // sync_timeout. Long enough for the nominal ~47 s declare with margin,
    // short enough that N>=500 stays well under the CI time guideline.
    double sync_time_budget_s = 75.0;
};

// One mission's result: every column of generated/wp5_campaign_runs.csv, in
// SI units. The compliance-metadata block is deterministic and passive (no
// legal analysis is performed).
struct RunResult {
    // identity / seeds
    std::string catalog;
    uint64_t    master_seed = 0;
    int         run_index   = 0;
    uint64_t    run_seed    = 0;

    // outcome + bookkeeping
    Outcome outcome            = Outcome::Other;
    int     removals           = 0;
    int     targets_attempted  = 0;
    double  dv_budget_m_s      = 0.0;
    double  dv_used_m_s        = 0.0;
    double  dv_remaining_m_s   = 0.0;
    int     kits_initial       = 0;
    int     kits_used          = 0;
    int     kits_remaining     = 0;
    double  sync_time_s        = 0.0;   // first successful sync; <0 => none (blank in CSV)
    double  mission_time_s     = 0.0;
    Outcome failure_reason     = Outcome::Completed;  // == outcome when not success
    bool    keep_out_violation = false;
    bool    abort              = false; // >=1 gate_abort event (feeds gate_abort_run_rate)
    bool    success            = false; // productive end: Completed or KitExhausted

    // per-target event tallies (feed the failure-classification counts)
    int gate_abort_events   = 0;
    int sync_timeout_events = 0;

    // realized-dispersion audit columns (WP5-native)
    double first_closing_speed_m_s = 0.0;  // capture closing speed of target 0 [m/s]
    double tumble_rate_deg_s       = 0.0;  // realized |w_t| of target 0 [deg/s]
    double solar_factor            = 0.0;  // realized atmospheric-density factor [-]

    // compliance metadata (passive, future-facing for WP7/WP8; no legal claim)
    bool        research_only                       = true;
    bool        class_level_preset                  = true;
    bool        uses_live_tle                       = false;
    bool        generates_target_specific_operations = false;
    bool        requires_owner_consent_assumption   = true;
    bool        owner_consent_assumed               = true;  // research assumption only
    std::string proximity_operations_mode           = "cooperative_consented_research";
    bool        unconsented_approach_blocked_by_policy = true;
    bool        controlled_reentry_mode             = false; // passive drag-sail baseline
    bool        rf_transmitter_modelled             = false;
    bool        remote_sensing_modelled             = false;
    std::string compliance_notes =
        "research-only class-level profile; WP5 performs no legal or regulatory "
        "determination";
};

// Run a single mission (deterministic given catalog + cfg + run_index).
RunResult run_one_mission(const DebrisCatalog& catalog, const CampaignConfig& ccfg,
                          const Config& base_cfg, int run_index);

// Run the full campaign for one catalog preset (n_runs missions).
std::vector<RunResult> run_campaign(const DebrisCatalog& catalog,
                                    const CampaignConfig& ccfg,
                                    const Config& base_cfg);

// One summary metric row (matches generated/wp5_campaign_summary.csv columns).
struct SummaryRow {
    std::string metric;
    double estimate   = 0.0;
    double wilson_low = 0.0;   // rates only; else 0
    double wilson_high = 0.0;  // rates only; else 0
    double p05 = 0.0, p50 = 0.0, p95 = 0.0;  // distributions only; else 0
    std::string units;
    std::string notes;
};

// Aggregate a catalog's runs into the ordered summary rows (rates with Wilson
// CIs, distribution percentiles, failure-classification counts).
std::vector<SummaryRow> summarize(const std::vector<RunResult>& runs);

// ------------------------------------------------------------------- output IO
// Serialize to the four committed WP5 artifacts under out_dir. `campaigns` is
// one (catalog, runs) pair per preset, in report order.
struct CatalogCampaign {
    DebrisCatalog          catalog;
    std::vector<RunResult> runs;
};
void write_runs_csv(const std::string& path, const std::vector<CatalogCampaign>& campaigns);
void write_summary_csv(const std::string& path, const CampaignConfig& ccfg,
                       const std::vector<CatalogCampaign>& campaigns);
void write_summary_md(const std::string& path, const CampaignConfig& ccfg,
                      const std::vector<CatalogCampaign>& campaigns);
void write_schema_md(const std::string& path);

}  // namespace adsc
