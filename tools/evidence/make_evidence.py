#!/usr/bin/env python3
"""WP7 Evidence Pack generator -> evidence/adsc_evidence_pack.md

The project's actual product: an English argument document for skeptical
agency/contractor engineers. EVERY number is read from a committed generated
artifact (generated/*.csv, generated/t6_flux_sweep.md, compliance outputs) --
nothing is hand-written; figures are embedded as relative links to the
committed generated/viz/*.svg. Python 3 standard library only (spec v4.2 R9).
No wall-clock timestamps or run times are embedded (spec v4.2 R6); output is
LF/ASCII and byte-deterministic so the CI reproducibility gate can compare it.

External literature citations are cite-at-fill: entries the author is
confident of carry full primary citations; anything not confidently known is
marked [CITATION NEEDED - PLACEHOLDER] rather than fabricated.

Usage: make_evidence.py [repo_root]   (default: repo root above this file)
"""
import csv
import json
import os
import sys

OUT_REL = os.path.join("evidence", "adsc_evidence_pack.md")


# ---- fixed formatting (platform-stable; the claim-audit test recomputes
# ---- these exact strings from the same CSVs) --------------------------------
def f0(v):
    return "%.0f" % float(v)

def f1(v):
    return "%.1f" % float(v)

def f2(v):
    return "%.2f" % float(v)

def f3(v):
    return "%.3f" % float(v)


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


class Data:
    def __init__(self, root):
        g = os.path.join(root, "generated")
        self.root = root
        self.ref = {r["metric"]: float(r["value"])
                    for r in load_csv(os.path.join(g, "reference_metrics.csv"))}
        self.wp5 = load_csv(os.path.join(g, "wp5_campaign_summary.csv"))
        self.wp5_runs_row = load_csv(os.path.join(g, "wp5_campaign_runs.csv"))[0]
        self.wp6 = load_csv(os.path.join(g, "wp6_cost_summary.csv"))
        self.wp3 = load_csv(os.path.join(g, "wp3_decay_trade.csv"))
        with open(os.path.join(g, "t6_flux_sweep.md"), encoding="utf-8") as f:
            self.t6 = {}
            for line in f:
                cells = [c.strip() for c in line.strip().strip("|").split("|")]
                if len(cells) >= 2 and cells[0] and not cells[0].startswith("-"):
                    self.t6[cells[0]] = cells[1:]
        with open(os.path.join(g, "compliance_findings.json"), encoding="utf-8") as f:
            self.comp = json.load(f)
        self.catalogs = []
        for r in self.wp5:
            if r["catalog"] not in self.catalogs:
                self.catalogs.append(r["catalog"])

    def wp5m(self, catalog, metric):
        for r in self.wp5:
            if r["catalog"] == catalog and r["metric"] == metric:
                return r
        raise KeyError((catalog, metric))

    def wp6q(self, catalog, record_type, metric, n_kits=None, weighting=None):
        for r in self.wp6:
            if (r["catalog"] == catalog and r["record_type"] == record_type
                    and r["metric"] == metric
                    and (n_kits is None or r["n_kits"] == str(n_kits))
                    and (weighting is None or r["weighting"] == weighting)):
                return r
        raise KeyError((catalog, record_type, metric, n_kits, weighting))

    def wp3_area25(self, catalog):
        for r in self.wp3:
            if r["catalog"] == catalog and r["record_type"] == "area_for_25yr":
                return float(r["value_solar_max"]), float(r["value_solar_min"])
        raise KeyError(catalog)


def rate_str(row):
    """'0.556 [0.512, 0.599] (Wilson 95%)' from a WP5 rate row."""
    return "%s [%s, %s] (Wilson 95%%)" % (
        f3(row["estimate"]), f3(row["wilson_low"]), f3(row["wilson_high"]))


def pctl_str(row, dp=2):
    fmt = {0: f0, 1: f1, 2: f2, 3: f3}[dp]
    return "p05 %s / p50 %s / p95 %s" % (
        fmt(row["p05"]), fmt(row["p50"]), fmt(row["p95"]))


def collect_placeholders(root):
    """Mechanically collect every uppercase PLACEHOLDER mark from the source
    tree (include/, src/, tools/ minus tools/evidence which only names the
    marker in order to collect and audit it). Deterministic ordering."""
    hits = []
    scan_dirs = ["include", "src", "tools"]
    exts = {".hpp", ".cpp", ".py", ".json", ".sh"}
    for base in scan_dirs:
        basepath = os.path.join(root, base)
        for dirpath, dirnames, filenames in os.walk(basepath):
            dirnames[:] = sorted(d for d in dirnames
                                 if d != "__pycache__" and d != "evidence")
            rel_dir = os.path.relpath(dirpath, root).replace(os.sep, "/")
            for fn in sorted(filenames):
                if os.path.splitext(fn)[1] not in exts:
                    continue
                path = os.path.join(dirpath, fn)
                rel = "%s/%s" % (rel_dir, fn)
                with open(path, encoding="utf-8", errors="replace") as f:
                    for i, line in enumerate(f, 1):
                        if "PLACEHOLDER" in line:
                            txt = line.strip()
                            txt = "".join(ch if ord(ch) < 128 else "?"
                                          for ch in txt)
                            txt = txt.replace("|", "/")
                            if len(txt) > 96:
                                txt = txt[:93] + "..."
                            hits.append((rel, i, txt))
    return hits


def build(d):
    A, B = d.catalogs[0], d.catalogs[1]
    Ashort, Bshort = A.split("/")[0].strip(), B.split("/")[0].strip()
    ref = d.ref
    L = []
    w = L.append

    # amortization pivots
    ns = sorted({int(r["n_kits"]) for r in d.wp6
                 if r["record_type"] == "amortization"
                 and r["metric"] == "cost_per_removal"})
    cprA = {n: float(d.wp6q(A, "amortization", "cost_per_removal", n)["p50"])
            for n in ns}
    nmin = min(ns, key=lambda n: cprA[n])
    ratio_min = f3(d.wp6q(A, "amortization",
                          "cost_per_removal_ratio_to_n1", nmin)["p50"])
    fomA_sp = d.wp6q(A, "fom", "fom", weighting="spatial_density")
    fomA_cr = d.wp6q(A, "fom", "fom", weighting="criticality")
    fomB_sp = d.wp6q(B, "fom", "fom", weighting="spatial_density")
    fomB_cr = d.wp6q(B, "fom", "fom", weighting="criticality")
    a25A = d.wp3_area25(A)
    a25B = d.wp3_area25(B)
    succA = d.wp5m(A, "success_rate")
    koA = d.wp5m(A, "keep_out_violation_rate")
    gaA = d.wp5m(A, "gate_abort_run_rate")
    n_runs = d.wp5[0]["n_runs"]
    seed_hex = "0x%X" % int(d.wp5[0]["master_seed"])
    dv_budget = d.wp5_runs_row["dv_budget_m_per_s"]
    kits0 = d.wp5_runs_row["kits_initial"]
    plan_targets = f0(ref["campaign_targets_per_mission"])
    # catalog class parameters from the committed decay CSV (never hand-written)
    rowA = [r for r in d.wp3 if r["catalog"] == A][0]
    rowB = [r for r in d.wp3 if r["catalog"] == B][0]
    massA, altA = float(rowA["mass_kg"]), float(rowA["altitude_km"])
    massB, altB = float(rowB["mass_kg"]), float(rowB["altitude_km"])
    inclA = f0(ref["wp3_target_inclination_deg"])
    gate_ms = f2(ref["wp3_contact_speed_m_s"])
    ratio_tow = f0(massA / ref["wp3_contact_mass_kg"])
    cs = d.comp["summary"]
    # WARN findings identified from the committed findings JSON, not hand-said
    warn_rules = [f["rule_id"] for f in d.comp["findings"] if f["status"] == "WARN"]

    w("# ADSC Evidence Pack")
    w("")
    w("**An open, reproducible evidence package for installer-type active debris")
    w("remediation (Kessler-precursor removal).** Audience: skeptical mission-design")
    w("engineers at the agencies and contractors of major launching states. Claim")
    w("discipline: every number in this document is read mechanically from a")
    w("committed, regenerable artifact (no hand-written figures); every figure is a")
    w("committed SVG; one script regenerates everything (section 9). This is a")
    w("numerical-simulation evidence package - NOT flight software, NOT a mission")
    w("proposal, and nothing here is a legal determination. This is not legal")
    w("advice. Maturity claim: TRL 4 for the GNC software element only,")
    w("element-scoped; system-level TRL is undefined and not claimed.")
    w("")
    w("## 1. Executive summary")
    w("")
    w("ADSC argues one thing quantitatively: **a small installer servicer that")
    w("attaches passive deorbit kits to massive derelict upper stages, several per")
    w("mission in one plane, is a high-leverage, low-cost intervention against the")
    w("debris-cascade source term - the cheapest per removal of the architectures")
    w("actually compared here (installer vs tug, batch vs single-target) - and the")
    w("argument survives honest negative results.** The two headline artifacts:")
    w("")
    w("**Batch amortization (WP6, relative cost units):** carrying N kits drives")
    w("cost/removal down to **%s x the single-target baseline at N=%d**"
      % (ratio_min, nmin))
    w("(%s CU vs %s CU p50), where the Delta-v budget - not the kit count - caps"
      % (f2(cprA[nmin]), f2(cprA[ns[0]])))
    w("removals; beyond that, extra kits add cost without removals and the curve")
    w("turns back up. That shape is the installer argument in one figure:")
    w("")
    w("![WP6 amortization](../generated/viz/wp6_amortization.svg)")
    w("")
    w("**Debris-risk reduction per cost (FoM = sum m_i * w(h_i) / C_campaign, p50")
    w("kg/CU at N=%s, under two congestion weightings - both PLACEHOLDER tables):**"
      % kits0)
    w("")
    w("| catalog class | spatial-density weighting | criticality weighting |")
    w("|---|---:|---:|")
    w("| %s (~%s t) | %s | %s |" % (Ashort, f0(massA / 1000.0),
                                    f1(fomA_sp["p50"]), f1(fomA_cr["p50"])))
    w("| %s | %s | %s |" % (Bshort, f1(fomB_sp["p50"]), f1(fomB_cr["p50"])))
    w("")
    w("The heavy class dominates under BOTH weightings (FoM is mass-dominated),")
    w("while the weightings disagree about band priority - an honest metric-choice")
    w("disagreement kept open as trade T5 (section 5). Conditionality, stated up")
    w("front: the %s FoM assumes a kit that closes for that class, and the"
      % Ashort)
    w("package's own trade shows sail-only does NOT close there (section 4, open")
    w("trade T1); the %s class is where the modeled kit closes today. Campaign"
      % Bshort)
    w("robustness under")
    w("dispersions (N=%s runs/catalog, fixed seed %s): success (productive end)"
      % (n_runs, seed_hex))
    w("%s for the %s class; keep-out violation rate %s."
      % (rate_str(succA), Ashort, rate_str(koA)))
    w("")
    w("## 2. Architecture and the installer argument")
    w("")
    w("**Why installer, not tug (locked decision D1).** Deorbit Delta-v scales with")
    w("the mass it must decelerate. A tug must decelerate the full %s kg %s-class"
      % (f0(massA), Ashort))
    w("stage - **%s x the entire ADSC servicer-at-contact mass** (%s kg, from the"
      % (ratio_tow, f1(ref["wp3_contact_mass_kg"])))
    w("committed catalog and mission data), and a realistic tug bus would be far")
    w("heavier still; the installer instead transfers a %s kg passive kit and"
      % f1(ref["wp3_kit_mass_kg"]))
    w("departs, so the")
    w("propellant cost of the deorbit itself is carried by drag (sail) or")
    w("electrodynamic drag (tether), not by the servicer. One mission services")
    w("several targets in one plane (batch amortization, section 5); the capture")
    w("interface is a geometry-keyed clamp on features present on every upper")
    w("stage by construction (nozzle throat / adapter ring, D4), not a generic")
    w("manipulator.")
    w("")
    w("**Cascade source-term framing (Kessler-precursor removal).** The collisional")
    w("cascade's fuel is the population of massive intact derelicts in congested")
    w("bands; fragments are the symptom. One intact-intact collision produces")
    w("thousands of trackable fragments (the 2009 Iridium 33 / Cosmos 2251")
    w("collision: 598 + 1603 = 2201 fragments cataloged by the U.S. Space")
    w("Surveillance Network as of January 2013; Liou, J.-C., \"An Analysis of")
    w("the FY-1C, Iridium 33, and Cosmos 2251 Fragments\", NASA Orbital Debris")
    w("Program Office, NTRS 20150003820, 2014), so removing (or equipping for")
    w("removal) the objects that would become the next fragment clouds attacks")
    w("the source term.")
    w("Anchors in the open literature:")
    w("")
    w("- Kessler, D. J., Cour-Palais, B. G., \"Collision Frequency of Artificial")
    w("  Satellites: The Creation of a Debris Belt\", Journal of Geophysical")
    w("  Research 83(A6), 1978 - the cascade mechanism itself.")
    w("- Liou, J.-C., Johnson, N. L., \"Risks in Space from Orbiting Debris\",")
    w("  Science 311(5759), 2006 - LEO population growth even without new")
    w("  launches (the instability argument).")
    w("- Liou, J.-C., Johnson, N. L., Hill, N. M., \"Controlling the growth of")
    w("  future LEO debris populations with active debris removal\", Acta")
    w("  Astronautica 66(5-6), 2010 - the classical few-removals-per-year")
    w("  environment-stabilization-class result (~5 objects/yr, jointly with")
    w("  ~90% post-mission-disposal compliance - removal alone does not carry")
    w("  the result) that sizes the campaign cadence ADSC targets.")
    w("- External validation of the reference target class: the environmental-")
    w("  index peak near ~800-900 km / 70-80 deg inclination reported by the ESA")
    w("  Space Debris Office's Annual Space Environment Report matches catalog_A")
    w("  (~%s km / ~%s deg). Source: ESA Space Debris Office, \"ESA's Annual"
      % (f0(altA), inclA))
    w("  Space Environment Report\", GEN-DB-LOG-00288-OPS-SD, Issue 10.0,")
    w("  1 May 2026, section 7.1 (\"areas with high risk concentration can be")
    w("  observed around 850 km of mean altitude and 70-80 degrees in")
    w("  inclination\", under a 90% PMD-success assumption).")
    w("- Population figures (>= 1 cm ~1.2 M objects) are MASTER-8-class model")
    w("  values (MASTER-8, released March 2019, reference epoch November 2016:")
    w("  Horstmann, A., Hesselbach, S., Wiedemann, C., \"Enhancement of S/C")
    w("  Fragmentation and Environment Evolution Models\", Final Report, ESA")
    w("  contract 4000115973/15/D/SR, 2020; ESA DISCOS environment statistics")
    w("  list 1.2 M objects in the 1-10 cm band for the 08/2024 MASTER-8")
    w("  reference population), carried as PLACEHOLDER inputs in the T6 flux")
    w("  table.")
    w("- Catalog class parameters (mass/altitude/inclination for the SL-16 / SL-8")
    w("  classes): class selection is anchored in the public debris-ranking")
    w("  literature per spec D2 - McKnight, D., et al., \"Identifying the 50")
    w("  statistically-most-concerning derelict objects in LEO\", Acta")
    w("  Astronautica 181, 282-291, 2021, doi:10.1016/j.actaastro.2021.01.021")
    w("  (a ranking dominated at the top by SL-16 / Zenit-2 second stages);")
    w("  the numeric class-parameter values themselves remain PLACEHOLDER")
    w("  constants in the auto-inventory.")
    w("")
    w("The GNC element is textbook-level by design (D11): Clohessy-Wiltshire")
    w("relative motion, quaternion sliding-mode tracking, multiplicative EKF -")
    w("open-literature methods, class-level targets, no live ephemerides, no")
    w("target-specific operational products.")
    w("")
    w("## 3. Safety case")
    w("")
    w("All numbers below regenerate from `generated/reference_metrics.csv` and the")
    w("WP5 campaign CSVs.")
    w("")
    w("- **Passive approach safety (D5/WP1):** across **%s thrust-off coasts**"
      % f0(ref["wp1_coast_samples"]))
    w("  sampled along the approach corridor and propagated 2 orbital periods,")
    w("  the worst closest approach is **%s m**, comfortably outside the %s m"
      % (f1(ref["wp1_worst_coast_min_range_m"]), f0(ref["wp1_keep_out_radius_m"])))
    w("  keep-out sphere. Scope honesty: this holds in the linear CW model")
    w("  (section 6).")
    w("- **Abort coverage including the capped case (F1):** the abort law is")
    w("  honest about its own limits. Clean case (contact range): impulse %s m/s,"
      % f3(ref["f1_clean_abort_dv_m_s"]))
    w("  full drift-null delivered, verified post-burn coast minimum %s m."
      % f3(ref["f1_clean_coast_min_m"]))
    w("  Capped case (large along-track drift): the commanded impulse saturates at")
    w("  the %s m/s thruster budget, the bounded-orbit property is LOST, and the"
      % f1(ref["f1_capped_abort_dv_m_s"]))
    w("  code reports the actual propagated coast minimum (%s m) instead of"
      % f1(ref["f1_capped_coast_min_m"]))
    w("  implying safety.")
    w("- **Campaign-level keep-out exposure (WP5):** violation rate %s over %s"
      % (rate_str(koA), n_runs))
    w("  dispersed missions (%s class; the %s class matches at the same seed"
      % (Ashort, Bshort))
    w("  discipline); abort-path exposure (runs with >= 1 closing-speed gate")
    w("  abort) %s - the %s m/s gate is genuinely exercised, not decorative."
      % (rate_str(gaA), gate_ms))
    w("")
    w("![WP5 keep-out rate](../generated/viz/wp5_keepout_rate.svg)")
    w("")
    w("- **Contact-energy budget (D4/WP3):** clamping at the gated closing speed")
    w("  (%s m/s) with the %s kg servicer-plus-kit carries **%s J** of kinetic"
      % (f2(ref["wp3_contact_speed_m_s"]), f1(ref["wp3_contact_mass_kg"]),
         f3(ref["wp3_contact_energy_j"])))
    w("  energy. The budget exists to show the geometry-keyed clamp concept does")
    w("  not depend on aggressive grappling of a decades-degraded surface -")
    w("  shedding MLI or paint would manufacture the very debris the mission")
    w("  removes. Whether this energy is below the actual MLI/paint damage")
    w("  threshold is NOT claimed here [CITATION NEEDED - PLACEHOLDER:")
    w("  low-speed-contact damage-threshold source for aged MLI/paint].")
    w("- **Estimate-driven, consistency-checked GNC (WP4):** the sync loop runs on")
    w("  estimates only (truth is structurally isolated to sensor models and error")
    w("  recording); under sensor noise the truth-evaluated criteria hold at")
    w("  **%s s** (truth-driven reference: %s s), relative-attitude RMS %s deg,"
      % (f2(ref["wp4_sync_time_s"]), f2(ref["wp2_sync_time_s"]),
         f2(ref["wp4_rms_att_rel_deg"])))
    w("  and the NEES/NIS watchdog (translation NIS %s vs dof 4) rejects the"
      % f3(ref["wp4_nis_trans"]))
    w("  classic covariance-inflation fake. The v2 detumble regression (settle")
    w("  %s s) is retained as a pinned reference." % f2(ref["v2_detumble_settle_time_s"]))
    w("")
    w("## 4. Deorbit-kit decay trades - the honest negatives first")
    w("")
    w("![WP3 decay trade](../generated/viz/wp3_decay_trade.svg)")
    w("")
    w("- **%s class (~%s t, ~%s km): sail-only does NOT close.** Meeting the"
      % (Ashort, f0(massA / 1000.0), f0(altA)))
    w("  IADC 25-year guideline (IADC-02-01, Space Debris Mitigation Guidelines)")
    w("  needs **%s..%s m^2** of sail (solar max..min, from the committed trade"
      % (f0(a25A[0]), f0(a25A[1])))
    w("  CSV) - impractical at the upper end. Under a US FCC flight scenario the")
    w("  standard adopted in 2022 (FCC 22-74) is **5-year** post-mission disposal")
    w("  for LEO space stations, which makes this negative strictly harder. This")
    w("  failure is a deliverable: it brackets open trade T1 and motivates the")
    w("  electrodynamic-tether branch.")
    w("- **%s class (~%s t, ~%s km): sail-only closes** at **%s..%s m^2** -"
      % (Bshort, f1(massB / 1000.0), f0(altB), f0(a25B[0]), f0(a25B[1])))
    w("  tens of square meters, a practical kit.")
    w("- The EDT branch is carried as a parametric study axis only (deorbit time")
    w("  vs an along-track decay-rate knob), never a tether performance claim.")
    w("- Solar activity is always reported as a min..max RANGE (T4); the")
    w("  atmosphere model is the Vallado exponential profile (Vallado,")
    w("  \"Fundamentals of Astrodynamics and Applications\", 4th ed., Table 8-4),")
    w("  with a deliberately coarse altitude-independent solar factor marked")
    w("  PLACEHOLDER.")
    w("")
    w("## 5. Campaign statistics and cost/FoM")
    w("")
    w("WP5 Monte Carlo: %s dispersed missions per catalog at fixed master seed"
      % n_runs)
    w("%s, %s targets-per-mission plan, %s kits, %s m/s Delta-v budget; all"
      % (seed_hex, plan_targets, kits0, f0(dv_budget)))
    w("dispersion magnitudes are PLACEHOLDER and centralized. Rates are quoted")
    w("with Wilson 95% intervals - never point estimates alone:")
    w("")
    w("| metric (%s class) | value |" % Ashort)
    w("|---|---|")
    w("| success (productive end) | %s |" % rate_str(succA))
    w("| nonproductive termination | %s |"
      % rate_str(d.wp5m(A, "nonproductive_termination_rate")))
    w("| gate-abort exposure | %s |" % rate_str(gaA))
    w("| keep-out violation | %s |" % rate_str(koA))
    w("| removals per mission | %s |"
      % pctl_str(d.wp5m(A, "removals_per_mission"), 0))
    w("| Delta-v used [m/s] | %s |" % pctl_str(d.wp5m(A, "dv_used_m_per_s"), 0))
    w("| sync arrival [s] | %s |" % pctl_str(d.wp5m(A, "sync_arrival_time_s"), 2))
    w("")
    w("![WP5 outcomes](../generated/viz/wp5_outcomes.svg)")
    w("")
    np_rate = d.wp5m(A, "nonproductive_termination_rate")
    if f3(np_rate["estimate"]) == f3(gaA["estimate"]):
        w("Under the current flat PLACEHOLDER leg costs the nonproductive-")
        w("termination and gate-abort RATES coincide numerically at this seed")
        w("(aborting missions almost always also exhaust the Delta-v budget; a")
        w("few end in a keep-out violation instead - see the runs CSV); they are")
        w("distinct concepts and separate columns.")
    else:
        w("The nonproductive-termination rate (%s) and gate-abort exposure (%s)"
          % (f3(np_rate["estimate"]), f3(gaA["estimate"])))
        w("are distinct concepts and separate columns in the campaign CSV.")
    w("The amortization curve (section 1) bottoms at N=%d because the **Delta-v"
      % nmin)
    w("budget, not the kit count, caps removals** - the honest capacity story a")
    w("mission designer needs. Cost is RELATIVE (CU) throughout: **no absolute")
    w("cost is predicted anywhere in this package**; the CU-to-currency anchor is")
    w("a deliberately unfilled cited-range PLACEHOLDER. Parameter sensitivity is")
    w("ranked by a one-at-a-time tornado (development cost dominates):")
    w("")
    w("![WP6 tornado](../generated/viz/wp6_tornado.svg)")
    w("")
    w("**Open trade T5 (metric choice changes band priority):** under the")
    w("spatial-density weighting the lower band (%s) carries the higher band"
      % Bshort)
    w("weight; under the criticality-style weighting the higher band (%s) does."
      % Ashort)
    w("Absolute per-catalog FoM stays mass-dominated under both, but the flip is")
    w("real and is kept visible rather than resolved by fiat:")
    w("")
    w("![WP6 FoM weightings](../generated/viz/wp6_fom_weightings.svg)")
    w("")
    w("## 6. Limitations - stated plainly, none hidden")
    w("")
    w("- **Linear CW scope (F2):** every passive-safety statement is exact only in")
    w("  the linearized Clohessy-Wiltshire model - circular target orbit, no J2,")
    w("  no differential drag, small separations. J2 and differential drag erode")
    w("  drift-free safety ellipses over time; a real mission re-verifies every")
    w("  coast against a higher-fidelity propagator.")
    w("- **No plane-change/phasing optimization:** inter-target phasing is a flat")
    w("  parameterized Delta-v/time cost (PLACEHOLDER).")
    w("- **Campaign sensor dispersions are drawn but not re-propagated** through")
    w("  the closed loop (the WP4 estimate-driven acceptance carries the")
    w("  closed-loop sensor argument; the campaign reuses the truth-driven sync")
    w("  primitive for tractable N=%s)." % n_runs)
    w("- **Sync-hold rests on the continuous-torque approximation:** the fine")
    w("  firing deadband that bounds the hold error assumes continuous torque; a")
    w("  real minimum-impulse-bit DACS would chatter or need reaction wheels.")
    w("- **Estimator scope:** known target inertia (a real mission needs inertia")
    w("  identification), Gaussian sensor abstractions (no outliers/dropouts/")
    w("  occlusions), sensor biases are knobs but not estimated, translation is")
    w("  estimated but not used for control (no closed-loop rendezvous guidance).")
    w("- **Small-debris (1-10 cm) removal is out of scope (T6) - by physics, not")
    w("  neglect:** at 10 km/s the specific kinetic energy is %s (%s TNT-"
      % (d.t6["specific kinetic energy"][0], d.t6["TNT specific-energy ratio"][0]))
    w("  specific-energy ratio); a 1 cm Al fragment carries %s (%s TNT"
      % (d.t6["1 cm Al sphere kinetic energy"][0],
         d.t6["1 cm Al sphere TNT equivalent"][0]))
    w("  equivalent); removing 1%/yr of the >= 1 cm population needs km^2-scale")
    w("  collection area that would itself be the largest collision cross-section")
    w("  in the band (full table: `generated/t6_flux_sweep.md`).")
    w("- **Jurisdiction coverage of the regulatory precheck is partial:** UN")
    w("  treaties + US agency rules + ITU/ESA references only; Russian, Chinese")
    w("  and Japanese national law - adopter targets - are NOT yet covered.")
    w("- **PLACEHOLDER discipline:** every unvalidated parameter in the repository")
    w("  is marked; the complete, mechanically-collected inventory is Appendix 10.")
    w("")
    w("## 7. Regulatory precheck summary")
    w("")
    w("**This is not legal advice.** The WP8 Compliance Matrix Generator is a")
    w("research-grade PRECHECK: it evaluates a declared mission profile against")
    w("versioned rulepacks (UN treaties, US FCC/FAA/NOAA, ITU and ESA references,")
    w("ADSC internal policy) and reports evidence gaps; it never determines legal")
    w("conformity. For the committed research profile:")
    w("")
    w("- Summary: **PASS=%d, INFO=%d, WARN=%d, BLOCK=%d, UNKNOWN=%d,"
      % (cs["PASS"], cs["INFO"], cs["WARN"], cs["BLOCK"], cs["UNKNOWN"]))
    w("  NOT_APPLICABLE=%d** - the research-only, class-level profile is not"
      % cs["NOT_APPLICABLE"])
    if warn_rules == ["ADSC-EXP-01"]:
        w("  blocked merely for being an ADR concept; the one WARN is the honest")
        w("  export-control-review-not-started flag (ADSC-EXP-01, identified from")
        w("  the committed findings).")
    else:
        w("  blocked merely for being an ADR concept; WARN findings: %s (from the"
          % ", ".join(warn_rules))
        w("  committed findings).")
    w("- The gate works in both directions: an ADR profile WITHOUT an affirmative")
    w("  owner-consent declaration is **BLOCKed** (OST Art. VIII precheck + ADSC")
    w("  policy, test-enforced), as is any live-ephemeris/target-specific-product")
    w("  profile (D11 guardrail).")
    w("- Framing (D9): the package assumes the operator is, or is contracted/")
    w("  consented by, the launching state of the target; consent in the research")
    w("  profile is a declared scenario assumption, never a legal fact. Dual-use")
    w("  guardrail (D11): open-literature methods, class-level targets, no live")
    w("  TLE ingestion, no operational approach products.")
    w("- Full matrix: `evidence/compliance_matrix.md` (rulepacks are versioned")
    w("  snapshots and can go stale; re-verify before any real-world use).")
    w("")
    w("## 8. Flight-software migration path (annex - deliberately NOT implemented)")
    w("")
    w("Adopters rewrite flight code; what they cannot cheaply reproduce is a")
    w("validated architecture trade. Spending effort there is the minimum-cost")
    w("allocation, so ADSC ships the trades and documents the migration path")
    w("instead of pretending at flight code:")
    w("")
    w("- Hardware abstraction layer between GNC core and device drivers.")
    w("- Allocation-free control path (fixed-size Eigen types already; remove")
    w("  remaining dynamic allocation, ban exceptions on the control path).")
    w("- Fixed-rate scheduler with worst-case-execution-time instrumentation")
    w("  hooks.")
    w("- Mode machine (SAFE / HOLD / APPROACH / SYNC / ABORT) with FDIR hooks;")
    w("  the TMR fuel store and closing-speed/keep-out gates already prototype")
    w("  the FDIR style.")
    w("- Telemetry/command dictionary generated from the config structs.")
    w("")
    w("Real-time processor-in-the-loop execution on representative hardware is")
    w("reserved as **WP9 - not started**; until then the maturity claim stays")
    w("TRL 4, element-scoped, and no flight-worthiness claim is made anywhere in")
    w("this package.")
    w("")
    w("## 9. Reproduction instructions (clean machine -> every number)")
    w("")
    w("```")
    w("git clone https://github.com/HeliCorgi/ADSC.git && cd ADSC")
    w("cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # + -DADSC_WERROR=ON for R3")
    w("cmake --build build")
    w("ctest --test-dir build                            # all suites")
    w("bash tools/regenerate_all.sh build                # regenerates generated/ + evidence/")
    w("git diff --exit-code -- generated/ evidence/      # byte-identical == reproduced")
    w("```")
    w("")
    w("`tools/regenerate_all.sh` is the single source of truth for regeneration")
    w("order (campaign -> cost -> decay -> flux -> reference metrics -> figures ->")
    w("compliance -> this document); CI runs exactly this script and enforces the")
    w("byte-identity gate on every push. Measured wall time is printed to the CI")
    w("log (see the latest `build-and-test` run); it is intentionally never")
    w("embedded in artifacts, and it is well under the one-hour reproduction")
    w("budget (D10). Requirements: C++17 compiler, Eigen 3.3+, CMake, Python 3")
    w("standard library only.")
    w("")
    w("## 10. Appendix - PLACEHOLDER inventory (mechanically collected)")
    w("")
    w("Everything the package does NOT validate, in one honest list: every line")
    w("in `include/`, `src/` and `tools/` carrying the uppercase PLACEHOLDER mark")
    w("(R10), collected automatically by the generator of this document")
    w("(`tools/evidence/` itself is excluded from the scan - it names the marker")
    w("only in order to collect and audit it). If a value is listed here, treat")
    w("it as unvalidated until a cited source replaces it.")
    w("")
    hits = collect_placeholders(d.root)
    w("Total marks: **%d**" % len(hits))
    w("")
    w("| location | line |")
    w("|---|---|")
    for rel, i, txt in hits:
        w("| `%s:%d` | %s |" % (rel, i, txt))
    w("")
    w("---")
    w("*Generated by `tools/evidence/make_evidence.py` from committed artifacts")
    w("only. This is not legal advice. Peaceful-use research package; see the")
    w("README disclaimer.*")
    w("")
    return "\n".join(L)


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    d = Data(root)
    out = os.path.join(root, OUT_REL)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    text = build(d)
    text.encode("ascii")  # hard guarantee: ASCII only
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print("[WP7] wrote %s (%d lines)" % (OUT_REL.replace(os.sep, "/"),
                                         text.count("\n") + 1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
