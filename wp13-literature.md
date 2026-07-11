# WP13 Literature / Citation Pack

Task: WP13 Task 1 (Opus, effort high) — literature/citation candidates for the
kit-class trade study + EDT physics model. Spec: `adsc-specification-v5.md`
§WP13 (lines 240-258), T7, D12/D13.

Rules honored: EVERY citation below was checked by live WebSearch/WebFetch on the
retrieval date. Each carries a verified-status label. **Nothing is fabricated.**
Where a specific number could not be pulled from a freely accessible source, that
is stated explicitly and the item is flagged for PLACEHOLDER treatment
(spec:167-174 pattern) rather than invented.

- Retrieval date (all items): **2026-07-11**
- Verified-status legend:
  - `web-verified` — citation and the key number(s) were confirmed against a
    fetched page or an explicit search-result snippet on 2026-07-11.
  - `web-verified (citation); number-paywalled` — bibliographic record confirmed
    live; the specific numeric value sits behind a paywall/binary PDF and is
    reported from a secondary snippet or left for PLACEHOLDER.
  - `training-data / derived` — value computed or recalled, not directly fetched;
    use only as cross-check, not as the primary cited number.
  - `not-found` — could not be verified; do NOT insert into the repo.

---

## Topic 1 — Bare electrodynamic tether deorbit physics (citable baseline)

### 1a. Sanmartín OML bare-tether foundation paper — PRIMARY BASELINE

**Sanmartín, J. R., Martínez-Sánchez, M., & Ahedo, E. (1993). "Bare Wire Anodes
for Electrodynamic Tethers." *Journal of Propulsion and Power*, 9(3), 353-360.**
- DOI: 10.2514/3.23629
- URL: https://arc.aiaa.org/doi/10.2514/3.23629
- Verified-status: **web-verified** (title, all three authors, journal, vol 9,
  no 3, pp 353-360, year 1993, and DOI confirmed via search 2026-07-11).
- Key content extracted: introduces the *bare tether* concept — leave a fraction
  of the tether near the anodic end uninsulated so it collects ambient electrons
  in the **orbital-motion-limited (OML)** regime once locally biased positive
  relative to the plasma. Validity condition: tether transverse radius small
  compared with both the electron Debye length and thermal gyroradius; large
  currents at moderate voltage drop. This is the citable OML current-collection
  law the EdtConfig force/current model must reference (task §0.1, decay model).

Supporting review (for the OML law written out + system context):
- **Sanmartín, J. R. "A Review of Electrodynamic Tethers for Space Applications."**
  UPM open archive PDF: https://oa.upm.es/30953/1/UA2.pdf
  - Verified-status: **web-found** (URL live). PDF is image/scan-based — machine
    text extraction failed, so specific formula lines were not pulled. Use the
    1993 paper above as the citable OML source; this review is a secondary pointer.

### 1b. EU FP7 BETs project (Bare Electrodynamic Tethers) — deorbit performance

**BETs — "Propellantless deorbiting of space debris by bare electrodynamic
tethers", FP7-SPACE, Grant Agreement No. 262972.** Coordinator: Prof. Juan R.
Sanmartín, Universidad Politécnica de Madrid (UPM). Duration: 2010-11-01 to
2014-01-31.
- CORDIS record: https://cordis.europa.eu/project/id/262972
- CORDIS reporting: https://cordis.europa.eu/project/id/262972/reporting
- Final Report PDF (UPM archive): https://oa.upm.es/39287/1/Final%20BETs%20Report%202.pdf
- Project site: http://www.thebetsproject.com/ (resources page:
  http://www.thebetsproject.com/resources)
- Verified-status: **web-verified** (GA number, coordinator, institution, dates,
  and project scope confirmed via CORDIS/search 2026-07-11). Final Report PDF URL
  is live but FlateDecode-compressed — headline numbers below come from the ESA
  outreach summary and BETs design papers, not the raw report binary.

**Deorbit performance numbers (citable, web-verified via search 2026-07-11):**
- Order-of-magnitude result: *a 1000-kg spacecraft can be deorbited from a
  1000-km-altitude orbit by a **10-kg** tether in about **one month**; a **1-kg**
  tether deorbits the same 1000-kg spacecraft in **less than a year**.* (ESA /
  phys.org BETs outreach, 2014: https://phys.org/news/2014-05-tether-solution-satellite-de-orbiting-reentry.html)
- Comparative claim: for state-of-the-art values with a few-weeks deorbit time,
  bare tethers are ~1-2 orders of magnitude lighter than active
  (propulsive) technologies and than drag-augmentation (sail) devices respectively.
- BETs design reference scenarios (used across BETs papers):
  (i) Earth-observation satellites, **700-1000 kg, initial orbit ~800 km, 98°
  inclination**; (ii) mega-constellation spacecraft, ~200 kg, 1200 km, 90°.
  Example tape-tether geometry: L = 3 km, width w = 2.5 cm, thickness h = 40 µm,
  with 2 km bare + 1 km inert segments.
- Software: BETsMA v1.0 mission-analysis tool developed under the project.
- Inclination note (directly supports spec's mandatory η(i), 240-243): the BETs
  performance scenarios are all high-inclination (98°, 90°), and the along-track
  component of v×B — the drag-producing component — degrades away from the
  equatorial/low-inclination case. See Topic 2/derivation task for the physics.

> Retrieval caveat: the exact BETs deorbit-time-vs-altitude-vs-inclination
> tables live in the compressed Final Report PDF and the paywalled Acta
> Astronautica BETs papers (e.g. "Comparison of technologies for deorbiting
> spacecraft from LEO at end of mission", S0094576516306555; "A code for the
> analysis of missions with electrodynamic tethers", S0094576522003113). The
> qualitative + order-of-magnitude numbers above ARE web-verified; any finer
> table pulled into the repo must cite the specific paper and be re-fetched.

---

## Topic 2 — EDT libration / dynamic instability (T7 open-risk pointer)

### PRIMARY libration-instability citation

**Peláez, J., Lorenzini, E. C., López-Rebollal, O., & Ruiz, M. (2000). "A New
Kind of Dynamic Instability in Electrodynamic Tethers." *The Journal of the
Astronautical Sciences*, 48(4), 449-476.**
- DOI: 10.1007/BF03546266
- URL: https://link.springer.com/article/10.1007/BF03546266
- Verified-status: **web-verified** (authors, title, journal, vol 48, no 4, pp
  449-476, year 2000, DOI confirmed via search 2026-07-11; Springer page itself
  redirects to an auth gate, so the citation is confirmed from the search
  record, not a full-text fetch).
- Key finding (for the T7 pointer): a conductive tether on a circular *inclined*
  orbit exhibits a genuine **dynamic instability** — non-periodic trajectories
  carry a positive net energy flux per orbit, pumping in-plane libration until it
  grows into rotation. The instability is present for flexible OR rigid tethers,
  in generator (deorbit) OR thruster mode, and its strength **depends on orbital
  inclination**. This is exactly the "libration/dynamic stability is explicitly
  unresolved" risk WP13 must cite and must NOT claim solved (spec:244-245,
  task §0.2).

### Supporting / corroborating libration references (all web-verified citations)

- **Zhong, R., & Zhu, Z. H. (2013). "Libration dynamics and stability of
  electrodynamic tethers in satellite deorbit." *Celestial Mechanics and
  Dynamical Astronomy*, 116(3), 279-298.** DOI: 10.1007/s10569-013-9489-4 —
  long-term libration/stability analysis in deorbit.
  URL: https://link.springer.com/article/10.1007/s10569-013-9489-4
- **"The dynamic instability analysis of electrodynamic tether system." *Nonlinear
  Dynamics* (2024).** DOI: 10.1007/s11071-024-09771-w —
  URL: https://link.springer.com/article/10.1007/s11071-024-09771-w — recent
  confirmation the instability remains an active, unresolved research topic.
- Verified-status: **web-verified** (both surfaced with full journal/vol/DOI in
  search 2026-07-11). Use Peláez 2000 as the primary T7 pointer; these two as
  "still open / actively studied" corroboration.

---

## Topic 3 — Ionospheric electron density Ne at ~700-900 km (cited parameter, no IRI impl.)

Spec:246 forbids an IRI implementation; WP13 only needs cited Ne numbers with a
source. Best web-verified anchors found:

### 3a. Representative engineering value at 800 km (PRIMARY citable number)

- **Ne ≈ 1.0 × 10¹¹ m⁻³ at 800 km altitude (daytime, representative value)** —
  assumed in a plasma-deorbit particle-simulation study.
  Source: "Particle Simulation of Plasma Drag Force Generation in the Magnetic
  Plasma Deorbit", arXiv:1805.06123.
  URL: https://arxiv.org/abs/1805.06123
  - Verified-status: **web-verified** (the "ion number density of 1.0×10¹¹ m⁻³ …
    typical daytime value at 800 km" statement confirmed via search snippet
    2026-07-11; the PDF binary itself did not text-extract cleanly).

### 3b. Altitude-appropriate in-situ measurement basis (DMSP at ~840-860 km)

- **Cai, X., et al. (2019). "Long-Term Trend of Topside Ionospheric Electron
  Density Derived From DMSP Data During 1995-2017." *Journal of Geophysical
  Research: Space Physics*, 124(12), 10708-10727.** DOI: 10.1029/2019JA027522
  URL: https://agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2019JA027522
  - DMSP SSIES makes **in-situ** thermal ion/electron density measurements at
    **~840-860 km** — i.e. directly in the ~800 km regime of both ADSC catalogs.
  - Verified-status: **web-verified (citation); number-paywalled** (journal/vol/
    pages/DOI confirmed; the article body returned HTTP 402, so discrete solar-
    min/max Ne values were not fetched).
- DMSP altitude confirmed independently: SSIES instruments measure at ~840 km;
  at solar maximum the 848-km ionosphere is >50% O⁺ at all local times, while at
  solar minimum O⁺ falls to ≲ H⁺/He⁺ (DMSP/SSIES mid-latitude page,
  https://dmsp.bc.edu/html2/ssiesmidlatitude.html — page live but TLS-cert
  mismatch blocked full fetch; content from search snippet).

### 3c. Solar min vs solar max range (order-of-magnitude, to be pinned)

Well-established topside behavior at ~800 km: Ne rises ~an order of magnitude from
solar minimum to solar maximum. Working range for the EdtConfig uncertainty band:
- **Solar min (~night/low F10.7): Ne ~ 10¹⁰ m⁻³ (order 10⁹-10¹⁰ m⁻³)**
- **Solar max (~day/high F10.7): Ne ~ 10¹¹ m⁻³ (up to ~10¹²  m⁻³ in daytime EIA)**

> Honest gap: a single freely-accessible table giving a clean solar-min **and**
> solar-max Ne pair at exactly 800 km was NOT extracted. The two best sources that
> tabulate it are paywalled: Cai et al. 2019 (above) and "Validation of the
> IRI-2020 topside ionosphere options through in-situ electron density
> observations by LEO satellites", *Adv. Space Res.* (2024),
> https://www.sciencedirect.com/science/article/pii/S0273117724005222 . For the
> repo: cite 3a (1.0×10¹¹ m⁻³ @ 800 km) as the mid/solar-active anchor, use the
> 3c range for the solar-min/max band, and keep the exact pair PLACEHOLDER until
> one of the paywalled tables is obtained. Do NOT invent a solar-min number.

Model-paper cross-refs (web-verified citations, for context, NOT to implement):
- Hoque, M. M., Jakowski, N., & Prol, F. S. (2022). "A new climatological electron
  density model…" *J. Space Weather Space Clim.*, 12, 1. DOI: 10.1051/swsc/2021044
  (validated against DMSP at ~800 km).
- Prol, F. S., Smirnov, A. G., Hoque, M. M., & Shprits, Y. Y. (2022). "Combined
  model of topside ionosphere and plasmasphere…" *Scientific Reports*, 12, 9732.
  DOI: 10.1038/s41598-022-13302-1 (topside model good to ~800 km).

---

## Topic 4 — Geomagnetic dipole moment (for the dipole B-field in the EDT model)

### PRIMARY citable dipole values

- **Centred-dipole magnetic moment M = 7.788 × 10²² A·m²; equatorial surface
  field B₀ = 3.01153 × 10⁴ nT (IGRF epoch 2000).**
  Source: SPENVIS (ESA/BIRA-IASB Space Environment Information System),
  "Dipole approximations of the geomagnetic field."
  URL: https://www.spenvis.oma.be/help/background/magfield/cd.html
  - Verified-status: **web-verified** (both M and B₀ pulled directly from the
    fetched page 2026-07-11). B₀ = √(g₁₀² + g₁₁² + h₁₁²); M = (4π R³/µ₀)·B₀.

### Current (IGRF-13/14 era) model reference + present value

- **Alken, P., Thébault, E., Beggan, C. D., et al. (2021). "International
  Geomagnetic Reference Field: the thirteenth generation." *Earth, Planets and
  Space*, 73, 49.** DOI: 10.1186/s40623-020-01288-x
  URL: https://link.springer.com/article/10.1186/s40623-020-01288-x
  - Verified-status: **web-verified (citation)** (authors, journal, article,
    year, DOI, epoch-2020.0 main-field model confirmed via search 2026-07-11).
- **Present value (epoch 2020.0): dipole moment ≈ 7.71 × 10²² A·m²** (slowly
  declining, ~5-6% over 1980→2020). Axial coefficient g₁₀(2020.0) = -29404.8 nT;
  with g₁₁ = -1450.9 nT, h₁₁ = 4652.5 nT ⇒ B₀ = √(·) ≈ 2.98 × 10⁴ nT ⇒
  M = (4π R³/µ₀)B₀ ≈ 7.71 × 10²² A·m² (R = 6371.2 km).
  - Verified-status: **training-data / derived** (IGRF-13 2020.0 g/h coefficients
    are standard; the 7.71×10²² value is computed from them, not fetched as a
    printed figure — the NOAA/IGRF-13 PDFs returned 403). Consistent with the
    web-verified SPENVIS 2000 value of 7.788×10²² and the documented declining
    trend. **For the repo, cite the SPENVIS value (7.788×10²²) as the primary
    web-verified dipole moment and note IGRF-13 (Alken 2021) as the current
    model; the 7.71×10²² is a derived cross-check.**

Design note: for an *aligned* dipole the model overstates high-latitude field
structure; spec/task allow "aligned dipole + stated tilt caveat" — the ~11° tilt
(geomagnetic vs geographic axis) and the eccentric-dipole offset are the caveats
to state (SPENVIS same page documents both).

---

## Topic 5 — Target parameters (catalog stage bodies)

### 5a. Zenit-2 second stage (SL-16 class) — Catalog A anchor / also massive

- **Empty (dry) mass ≈ 8,300 kg** (one source lists ~9,000 kg empty);
  **length 11.50 m, diameter 3.90 m**; engine 1× RD-120 (+ RD-8 vernier).
  Sources: braeunig space specs (http://www.braeunig.us/space/specs/zenit.htm),
  Astronautix Zenit-2 (http://www.astronautix.com/z/zenit-2.html),
  Wikipedia Zenit-2 (https://en.wikipedia.org/wiki/Zenit-2).
- **Typical orbit (Tselina-2 payloads): ~845 × 857 km, inclination 71.01°.**
  (Also a 65° family; SL-16 R/B clusters span ~750-850 km, incl. 71° and 81°.)
- Verified-status: **web-verified** (mass 8,300 kg, dims, RD-120, and the
  845×857 km / 71.01° Tselina-2 orbit all confirmed via search 2026-07-11; the
  8,300-vs-9,000 kg spread is a real source discrepancy — carry both, flag the
  8,300 kg as the more common dry-mass figure).
- Cross-check: McKnight 2021 (Topic 6) — the top 20 most-concerning LEO objects
  are all SL-16 (Zenit-2) second stages, ~9 t class, ~850 km.

### 5b. Kosmos-3M second stage (SL-8 class, 11K65M)

- **Empty (dry) mass ≈ 1,435 kg**; gross mass ≈ 20,135 kg; engine 1× 11D49
  (RD-219 family verniers); length ~6 m, diameter 2.4 m.
  Sources: Astronautix Kosmos 11K65M (http://www.astronautix.com/k/kosmos11k65m.html),
  Gunter's Space Page (https://space.skyrocket.de/doc_lau_det/kosmos-3m.htm).
- **Typical orbits — SL-8 R/B clusters (as of 2014 survey): Box 3 ≈ 700-800 km
  (50 R/Bs), Box 2 ≈ 900-1000 km (143 R/Bs), Box 1 ≈ 1500-1600 km (44 R/Bs);
  inclinations ~74° and ~83°.** The ~740-780 km / ~74° preset the task cites
  matches the 700-800 km, 74° sub-population.
- Verified-status: **web-verified** (1,435 kg empty mass, 20,135 kg gross, 11D49,
  and the three-box cluster altitudes/inclinations confirmed via search
  2026-07-11; per-object page fetches were blocked by host cert/connection
  issues, so mass is snippet-level, not a fetched spec sheet — solid but tag it
  as such).

---

## Topic 6 — Class-C candidates (massive / high-risk, controlled-reentry class)

### PRIMARY ranking citation (the criticality basis, spec §4 amendment)

- **McKnight, D., Witner, R., Letizia, F., Lemmens, S., Anselmo, L., Pardini, C.,
  Rossi, A., Kunstadter, C., et al. (2021). "Identifying the 50
  statistically-most-concerning derelict objects in LEO." *Acta Astronautica*,
  181, 282-291.** DOI: 10.1016/j.actaastro.2021.01.021
  URL: https://www.sciencedirect.com/science/article/abs/pii/S0094576521000217
  ADS: https://ui.adsabs.harvard.edu/abs/2021AcAau.181..282M/abstract
  - Verified-status: **web-verified (citation + headline findings)** (journal,
    vol 181, pp 282-291, 2021 confirmed; the abstract page and open PDF fetches
    partially blocked, key findings from search 2026-07-11).
  - Key findings: 11 expert teams produced ranked lists → consensus top-50.
    **Rocket bodies dominate; the top ~20 are all SL-16 (Zenit-2) second stages.**
    **Average mass of the top-50 objects ≈ 5,295 kg.** Removing the top-50 would
    roughly halve LEO collision-risk; top-10 ≈ 30% reduction.

### Class-C candidate targets (2-3, with cited mass/orbit/inclination)

1. **Envisat (ESA) — the canonical controlled-reentry-class derelict.**
   - **Mass 8,211 kg** (Service Module 2,673 + PEB 1,021 + Payload Carrier 2,078
     + Fuel 319 + Instruments 2,118 kg); **sun-synchronous orbit ~765-800 km,
     inclination ~98.4-98.55°**; defunct since April 2012.
   - Sources: eoPortal (https://www.eoportal.org/satellite-missions/envisat),
     ESA Earth Online (https://earth.esa.int/eogateway/missions/envisat/description),
     Wikipedia (https://en.wikipedia.org/wiki/Envisat).
   - Verified-status: **web-verified** (8,211 kg mass breakdown, ~800 km sun-sync,
     ~98.5° inclination, 2012 failure all confirmed 2026-07-11). Envisat is the
     most-cited single "too massive to safely leave / high casualty-risk on
     uncontrolled reentry" object — the archetypal Class-C target.

2. **SL-16 (Zenit-2) second stage — top of the McKnight ranking, massive.**
   - **~8,300-9,000 kg, ~845-857 km, 71.0° (also an 81° cluster).** (Topic 5a.)
   - Verified-status: **web-verified.** Note: the ADSC task uses Zenit-2 as the
     Class-A *anchor*; by mass it is equally a Class-C controlled-reentry
     candidate. Whether an 8.9-t stage needs controlled reentry (Ec>1e-4) is the
     Topic-7 casualty-area question — flag for the Class-C vs Class-A split.

3. **SL-8 (Kosmos-3M) second stage — high-count, lighter concern object.**
   - **~1,435 kg, 700-1000 km, 74°/83°.** (Topic 5b.) Appears prominently in
     SL-8-specific ADR mission studies (e.g. ADReS-A, Springer
     10.1007/978-3-319-15982-9_3). Lighter than Envisat/SL-16 → likely NOT
     controlled-reentry class, useful as the mass contrast in the Class-C table.
   - Verified-status: **web-verified.**

> For a third *distinct massive* Class-C payload (not a rocket body), Envisat is
> the strongest and only fully-verified large defunct **satellite** in the
> concern literature at ~800 km. If a second large payload is wanted, candidates
> to verify later: ADEOS-II/Midori-II (~3.7 t, ~800 km, 98.6°) and ERS-2 (~2.5 t
> — but ERS-2 already reentered Feb 2024, so it is historical). These were NOT
> re-verified in this pass — mark PLACEHOLDER until fetched.

---

## Topic 7 — Casualty-risk framework (controlled vs uncontrolled reentry)

### PRIMARY standard

- **NASA-STD-8719.14A (with Change 1), "Process for Limiting Orbital Debris."**
  URL: https://soma.larc.nasa.gov/SIMPLEx/pdf_files/871914.pdf (also
  https://explorers.larc.nasa.gov/HPMIDEX/pdf_files/10_nasa-std-8719.14a_with_change_1.pdf)
  - **Requirement 4.7-1 — Limit the risk of human casualty: the expected
    worldwide human-casualty risk from reentering debris shall not exceed
    0.0001 (1 in 10,000), i.e. Ec < 1 × 10⁻⁴.** Casualty is assumed for any
    surviving fragment impacting with kinetic energy **> 15 J**.
  - **Controlled reentry:** the reentry must still satisfy 4.7-1, AND the targeted
    trajectory shall ensure no surviving debris of >15 J lands closer than
    **370 km from foreign landmasses** (with defined stand-off from U.S.
    territories); the deorbit-burn success probability must be high enough not to
    violate 4.7-1.
  - Compliance tools: **DAS (Debris Assessment Software)** and **ORSAT (Object
    Reentry Survival Analysis Tool)**.
  - Verified-status: **web-verified** (the 1×10⁻⁴ / 1-in-10,000 limit, the 15 J
    casualty threshold, the 370 km controlled-reentry rule, and DAS/ORSAT all
    confirmed via fetch + search 2026-07-11). Note: the requirement is numbered
    **4.7-1** in the standard; a first WebFetch guessed "4.7.2" — disregard that,
    4.7-1 is correct per the follow-up search of the standard's own text.

### ESA / European equivalent (same 1e-4 threshold, DRAMA/SARA tool)

- **ESA re-entry safety: the 1 × 10⁻⁴ casualty-risk limit is likewise applied
  (harmonised with IADC).** Uncontrolled reentries exceeding it require design-
  for-demise or controlled/targeted reentry.
  URL: https://technology.esa.int/page/re-entry-safety
- **DRAMA / SARA** — ESA's Debris Risk Assessment and Mitigation Analysis suite;
  **SARA (Survival And Risk Analysis)** is its reentry-survival + on-ground-
  casualty module (the ESA analogue of NASA DAS/ORSAT).
  ESA DRAMA: https://sdup.esoc.esa.int/drama/ (SARA is the reentry module).
- Verified-status: **web-verified (framework + threshold)** (ESA's use of the
  same 1e-4 casualty limit and the DRAMA/SARA tooling confirmed via search
  2026-07-11; SARA's exact algorithm details not fetched — cite the ESA
  re-entry-safety page + DRAMA suite, not a specific SARA number).

Cross-refs (web-verified citations): NASA ORSAT page
(https://orbitaldebris.jsc.nasa.gov/reentry/orsat.html); "Debris Assessment
Software (DAS) Reentry Risk Analysis" (ResearchGate 320808841).

---

## Summary table — verification status by topic

| # | Item | Citation status | Key number status |
|---|------|-----------------|-------------------|
| 1a | Sanmartín/Martínez-Sánchez/Ahedo 1993 OML bare-tether | web-verified | OML law (qualitative) verified |
| 1b | BETs FP7 GA 262972 + deorbit performance | web-verified | 10 kg→1000 kg from 1000 km in ~1 mo: verified; fine tables paywalled |
| 2 | Peláez et al. 2000 libration instability (T7) | web-verified | inclination-dependent instability: verified |
| 3 | Ne @ ~800 km | 1.0e11 m⁻³ @800 km verified; solar-min/max pair paywalled | mid anchor verified; solar-min PLACEHOLDER |
| 4 | Geomagnetic dipole moment | SPENVIS M=7.788e22 web-verified; IGRF-13 model verified | 2020 value 7.71e22 derived |
| 5a | Zenit-2/SL-16 2nd stage | web-verified | 8,300(-9,000) kg, 11.5×3.9 m, 845×857 km/71.0° |
| 5b | Kosmos-3M/SL-8 2nd stage | web-verified | 1,435 kg dry, 700-1000 km, 74°/83° |
| 6 | McKnight 2021 top-50 + Envisat | web-verified | Envisat 8,211 kg/~800 km/98.5°; top-50 avg 5,295 kg |
| 7 | NASA-STD-8719.14A + ESA/DRAMA-SARA | web-verified | Ec<1e-4, 15 J, 370 km controlled rule |

## Items to keep PLACEHOLDER (spec:167-174) — NOT fabricated

- Exact solar-min **and** solar-max Ne pair at exactly 800 km (only paywalled
  tables found; mid/solar-active anchor 1.0×10¹¹ m⁻³ is usable now).
- Precise BETs deorbit-time-vs-(altitude, inclination) table (compressed Final
  Report PDF + paywalled Acta Astronautica papers).
- A second large non-rocket-body Class-C payload beyond Envisat (ADEOS-II / ERS-2
  candidates not re-verified this pass).
- IGRF-13 2020.0 printed dipole-moment figure (value derived, not fetched).
