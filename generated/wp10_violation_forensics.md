# WP10c keep-out violation forensics (ARCHIVE: legacy abort law, ds-v1)

**Supersession note (WP11).** This is the WP10c forensic record of the
ds-v1 campaign under the LEGACY drift-null abort law
(`compute_safe_abort`). WP11 replaced the campaign's screened abort with
the clearing-abort law (`compute_clearing_abort`); the committed
campaign CSV no longer exhibits these violations (see
`wp11_abort_audit.md` for the current keep-out result and the R15
changelog in the evidence pack for the archived rates). This archive
regenerates deterministically from the pinned seed recipe plus the
legacy law, which remains in the codebase; the same 14 cases are pinned
in C++ in `tests/test_forensic14.cpp` (R15).

## 1. Replay recipe

- Per-run seed: `SplitMix64(master_seed XOR FNV1a64(catalog), run_index)`
  with `master_seed = 0x5AD5C0DECAFE2026` (campaign.hpp); the run seeds in
  the forensics table below equal the committed `run_seed` column.
- Draw stream: `std::mt19937_64(run_seed)`, 53-bit uniforms in (0, 1],
  explicit Box-Muller with spare caching, in the exact draw order of
  `run_one_mission` (campaign.cpp).
- Abort physics: LEGACY Clohessy-Wiltshire drift-null impulse capped at
  2.0 m/s (mission.cpp `compute_safe_abort`), RK4 coast at dt = 8 s over
  1 orbital period of the catalog altitude (the campaign's coarsened
  screen, main_campaign.cpp), keep-out radius 200 m, standoff 400 m.
- To reproduce: `python3 tools/forensics/make_forensics.py` from the repo
  root, or `bash tools/regenerate_all.sh build` for the full pipeline.

## 2. Replay cross-validation (gate for everything below)

- Stream-level columns (run_seed, solar_factor, first_closing_speed) of
  the legacy replay match the committed schema-1.1 CSV for all 1000 runs
  (these draws are abort-law-independent).
- Replayed under the LEGACY law, exactly 14 runs terminate in
  keep_out_violation (SL-16 / Zenit-2 second stage: 7, SL-8 / Kosmos-3M second stage: 7), matching the WP10c
  classification and the C++ forensic-14 pin set; every other run's full
  bookkeeping is identical under both laws (their aborts clear either
  way). The clearing-law replay of the committed campaign is separately
  hard-gated in `wp11_abort_audit.md` section 2.
- Modeled assumption, guarded: every sync attempt succeeds (the committed
  CSV contains 0 sync_timeout_events across all 1000 runs).

## 3. Findings (WP10c, archived)

**The D13 working hypothesis is REFUTED by the data.** D13 hypothesized the
primary mechanism as *capped-abort residual drift* re-entering the keep-out
sphere. In fact **all 14 violations are CLEAN aborts**: the commanded
drift-null impulse (max 0.306 m/s across the violating runs) never reaches
the 2.0 m/s cap, and the post-burn secular drift term |vy + 2 n x| is
numerically zero for every violating run (column `post_burn_drift_m_per_s`).
Capped aborts observed among the violations: 0.

**Actual mechanism -- clean-abort safety-ellipse keep-out intersection.**
The legacy abort law nulls the drift, which yields a *bounded* relative
orbit, not necessarily a *clearing* one. The post-burn coast is the
closed-form ellipse `x(t) = x0 cos(nt)`, `y(t) = y0 - 2 x0 sin(nt)`,
`z(t) = z0 cos(nt)`: a dispersed radial offset x0 produces an along-track
swing of 2|x0| about the dispersed standoff y0, so the coast approaches
the target to roughly `|y0| - 2|x0|` (exactly: the sampled ellipse
minimum, column `analytic_ellipse_min_range_m`, which matches the RK4
sweep within the 8 s screen resolution). Every violating run has |x0|
between 62.6 m and 142.4 m (1.6-3.6 sigma of the 40 m per-axis dispersion),
and the coast minimum occurs at a quarter orbit (x0 < 0) or
three-quarter orbit (x0 > 0), exactly as the ellipse geometry predicts.
The violation is a *geometric* property of where the abort starts, not a
thruster-authority (cap) property. **WP11 addressed exactly this
mechanism** with the clearing-abort law's analytic-clearance acceptance
(named design element per the WP11 completion criteria).

Classification of all 14 violating runs:
`clean_abort_safety_ellipse_intersects_keep_out`: 14 of 14.
`capped_abort_residual_drift`: 0.
`starts_inside_keep_out`: 0.

Model scope: this analysis is [L0: linear CW coast, campaign keep-out
screen at dt = 8 s over 1 period, dispersion set ds-v1]; it inherits
every WP5 model limitation (sensor dispersions drawn but not
re-propagated; flat placeholder Delta-v costs; see the evidence-pack
inventory).

## 4. Per-run forensic table

Full precision in `wp10_violation_forensics.csv`; abridged view:

| catalog | run | tgt | x0 [m] | y0 [m] | z0 [m] | dv mag [m/s] | capped | coast min [m] | t_min [s] | ellipse min [m] | classification |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---|
| SL-16 | 65 | 3 | -109.2 | -391.0 | 47.6 | 0.229 | false | 172.5 | 1528 | 172.5 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-16 | 81 | 0 | -62.6 | -295.8 | 3.2 | 0.141 | false | 170.6 | 1528 | 170.6 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-16 | 86 | 1 | 88.9 | -377.7 | -36.5 | 0.208 | false | 199.9 | 4576 | 199.9 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-16 | 112 | 3 | 103.1 | -403.4 | -17.2 | 0.211 | false | 197.1 | 4576 | 197.1 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-16 | 340 | 1 | 142.4 | -444.1 | 39.6 | 0.285 | false | 159.4 | 4576 | 159.4 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-16 | 460 | 0 | -81.0 | -340.0 | -30.5 | 0.202 | false | 178.1 | 1528 | 178.1 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-16 | 490 | 1 | -136.5 | -403.0 | 31.6 | 0.306 | false | 130.0 | 1528 | 130.0 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 88 | 0 | -110.8 | -340.9 | -23.6 | 0.214 | false | 119.4 | 1496 | 119.4 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 99 | 0 | -78.3 | -281.4 | -6.7 | 0.167 | false | 124.8 | 1496 | 124.8 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 160 | 1 | -126.4 | -380.9 | 20.7 | 0.257 | false | 128.1 | 1496 | 128.1 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 254 | 1 | -136.6 | -326.5 | -67.5 | 0.283 | false | 53.3 | 1496 | 53.3 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 362 | 2 | -88.6 | -376.3 | 125.0 | 0.190 | false | 199.1 | 1496 | 199.1 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 383 | 2 | -81.7 | -346.4 | 39.6 | 0.161 | false | 183.0 | 1496 | 183.0 | clean_abort_safety_ellipse_intersects_keep_out |
| SL-8 | 419 | 1 | 107.8 | -403.6 | 85.7 | 0.242 | false | 188.1 | 4488 | 188.1 | clean_abort_safety_ellipse_intersects_keep_out |

## 5. Honest limits

- Linear CW only (L0); no J2/drag/SRP. The WP12 fidelity ladder re-issues
  safety statements per level.
- The 8 s screen step under-resolves the exact minimum by up to a few
  meters versus the analytic ellipse; the classification is unaffected.
- This archive CLASSIFIES the superseded legacy-law campaign; the
  committed campaign numbers are those of the WP11 clearing law (see
  wp5_campaign_summary.md and wp11_abort_audit.md).
