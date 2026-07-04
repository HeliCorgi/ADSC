# WP11 closed-loop translation guidance -- mode machine

Deterministic, truth-fed L0 demonstration of the WP11 closed-loop translation-guidance mode machine (D13/D5): it extends (never replaces) the WP3 installer-mission phase structure with two-impulse V-bar hops, a synchronization dwell, a glideslope-with-floor final approach, contact, and a retreat back to the standoff, all screened by the WP11 clearing-abort law's reachability rule at every committed impulse. [L0: linear CW, truth-fed guidance, deterministic]

## Mode table

| mode | entry | exit | abort condition | abort action |
|---|---|---|---|---|
| far_approach | start of the demo, at the far V-bar hold (guid_hold_far_m) | two-impulse CW transfer departs toward the mid hold | either impulse's post-state fails the reachability screen | execute the clearing abort from the current state and enter Abort |
| hold | arrival at the mid V-bar hold (guid_hold_mid_m) | two-impulse CW transfer departs toward the sync-hold standoff | either impulse's post-state fails the reachability screen | execute the clearing abort from the current state and enter Abort |
| sync_hold | arrival at the departure standoff (depart_standoff_factor x keep_out) | sync dwell (Config::sync_hold_s + a fixed sync allowance) completes | n/a (zero-dv equilibrium dwell; attitude sync is WP2's job) | n/a |
| final_approach | sync dwell complete | range <= guid_contact_range_m | a glideslope step's post-state fails the reachability screen, or the LOS cone is violated | execute the clearing abort from the current state and enter Abort |
| contact | range <= guid_contact_range_m | contact-speed matching burn commits | the matching burn's post-state fails the reachability screen | execute the clearing abort from the current state and enter Abort |
| retreat | immediately after contact | the standoff equilibrium is re-established (post-hop range >= standoff, or accepted in place) | n/a (the RetreatHop abort IS the escalation of last resort; not itself re-screened) | n/a |
| complete | standoff equilibrium re-established outside keep_out + margin | end of the demo | n/a | n/a |
| abort | a reachability screen or LOS check fails | n/a (terminal) | n/a | n/a (the last feasible clearing abort has already been executed) |

## Flown profile (this run)

| mode | t_start_s | t_end_s | dv_m_per_s | min_range_m | abort_feasible_throughout |
|---|---:|---:|---:|---:|---|
| far_approach | 0.000 | 1521.015 | 0.56193 | 800.000 | true |
| hold | 1521.015 | 3042.030 | 0.56193 | 400.000 | true |
| sync_hold | 3042.030 | 3147.030 | 0.00000 | 400.000 | true |
| final_approach | 3147.030 | 7167.030 | 0.95561 | 1.000 | true |
| contact | 7167.030 | 7167.030 | 0.05016 | 1.000 | true |
| retreat | 7167.030 | 11730.075 | 0.40398 | 1.000 | true |
| complete | 11730.075 | 11730.075 | 0.00000 | 400.000 | true |

Completed: true. Final mode: complete. Contact speed 0.1000 m/s (design 0.1000 m/s, gate 0.1500 m/s). Total Delta-v 2.5336 m/s. Min clearance outside keep-out 200.0000 m. Reachability screen held every step: true. LOS cone held throughout final approach: true.

Contact speed is produced by the guidance profile (v = max(floor, k*range)), not merely gated; the WP1-era known limit is closed at L0.

## Escalation design note

A pure along-track opening-drift burn (spend the full abort_dv budget as a single along-track impulse) was evaluated and REJECTED for the WP11 clearing-abort law's third escalation stage: its oscillation amplitude (~4*abort_dv/n, km-scale at LEO) swamps the ~100 m keep-out geometry, so the resulting coast swings back through near-zero range before the secular drift ever opens the gap -- measured coast minima of a few meters against a required >= 0.8*range, for either sign of the escalation. The two-impulse radial retreat hop (SafeAbort::Status::RetreatHop, mission.hpp) was adopted instead: it swaps the radial-offset sign via a half-period hop, landing on a drift-free ellipse whose analytic minimum clears keep-out + margin, while its own transient leg opens monotonically for the geometry where it is reached. This is the negative result required by spec v5 section 10: the rejected design is documented, not merely discarded.

[L0: linear CW, truth-fed guidance, deterministic]
