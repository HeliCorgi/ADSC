#pragma once

#include <vector>

#include <Eigen/Dense>

#include "adsc/mission.hpp"
#include "adsc/relmotion.hpp"

namespace adsc {

// ============================================================================
// Closed-loop translation guidance (WP11)
// ----------------------------------------------------------------------------
// Extends (never replaces) the WP3 installer-mission phase structure with a
// deterministic, truth-fed translation-guidance profile: two-impulse V-bar
// hops between passively-safe holds, a synchronization dwell at the
// departure standoff, a glideslope-with-floor final approach, contact, and a
// retreat back to the standoff. L0 fidelity (R14): linear CW dynamics, no
// sensor noise, no attitude dynamics (WP2's job) -- every impulse is applied
// to the truth relative state directly.
//
// Safety posture (D5/D13): every committed impulse is screened BEFORE it is
// applied (reachability_screen_ok, guidance.cpp). OUTSIDE the keep-out sphere
// the screen requires the post-impulse state to reach a bounded, margin-
// clearing abort -- Clean, BoundedClearing, or WP11's two-impulse RetreatHop
// (which now carries a genuine terminal bounded guarantee, unlike the
// rejected pure along-track opening-drift design -- see
// SafeAbort::Status::RetreatHop). INSIDE the sphere (final approach /
// contact, the authorized exception to the keep-out gate) the screen is
// retreat-feasibility: an abort must still carry a bounded terminal
// guarantee AND its transient coast must not come significantly closer than
// the range at which it is evaluated. If a screen ever fails, the guidance
// does not commit the impulse and instead executes the clearing abort from
// the CURRENT (already-committed) state and enters Abort mode -- a safety-
// logic branch the nominal L0 demo below is not expected to exercise.
// ============================================================================

enum class GuidanceMode {
    FarApproach,
    Hold,
    SyncHold,
    FinalApproach,
    Contact,
    Retreat,
    Complete,
    Abort
};

const char* guidance_mode_label(GuidanceMode m);

// One row of the mode-machine table (emitted to generated/wp11_guidance_modes.md
// by main_metrics.cpp) -- the documented machine, single source of truth for
// both the header comments above and that markdown.
struct GuidanceModeSpec {
    GuidanceMode mode;
    const char* entry;
    const char* exit;
    const char* abort_condition;
    const char* abort_action;
};
std::vector<GuidanceModeSpec> guidance_mode_table();

// One flown phase of a GuidedApproach run.
struct GuidedPhaseLog {
    GuidanceMode mode = GuidanceMode::FarApproach;
    double t_start_s = 0.0;
    double t_end_s = 0.0;
    double dv_m_s = 0.0;         // sum of |impulse| committed during this phase
    double min_range_m = 0.0;    // minimum range sampled during this phase
    bool   abort_feasible_throughout = true;  // WP11 screen held at every step of this phase
};

// Outcome of a full guided approach (WP11 L0 demo).
struct GuidedApproachReport {
    bool          completed  = false;
    GuidanceMode  final_mode = GuidanceMode::Abort;
    double        contact_speed_m_s = 0.0;  // achieved by guidance at contact range
    double        contact_range_m   = 0.0;
    double        dv_total_m_s      = 0.0;
    // min (range - keep_out) over all pre-authorization (outside-sphere)
    // flight (samples inside the sphere -- final approach's tail, contact,
    // the inside portion of retreat -- are excluded, that penetration being
    // the authorized exception, D5/WP11).
    double        min_clearance_outside_m = 0.0;
    bool          abort_feasible_every_step = false;  // reachability screen held at every guidance step and hold
    bool          los_cone_ok = true;
    std::vector<GuidedPhaseLog> phases;
};

// Deterministic L0 closed-loop translation-guidance demo (WP11): flies the
// far-approach -> hold -> sync-hold -> final-approach -> contact -> retreat
// sequence under the reachability screen and reports the flown profile.
class GuidedApproach {
public:
    explicit GuidedApproach(const Config& cfg);

    GuidedApproachReport fly();

private:
    Config  cfg_;
    CwModel cw_;
};

}  // namespace adsc
