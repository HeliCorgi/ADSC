# validation/RESULTS.md -- independent cross-validation results

Run 2026-07-12, Python 3.11.9 (Windows), pinned versions per
`requirements.txt`: numpy 1.26.4, scipy 1.17.1, hapsira 0.18.0,
astropy 6.1.7, pymsis 0.12.0. Deterministic, no RNG. All numbers below
are real script output (`check_cw.py`, `check_decay.py`,
`check_edt_eta.py`), not hand-edited -- every number in this file appears
verbatim in one of these scripts' stdout or one of the raw CSVs listed
below (re-run the three scripts and diff against the committed CSVs to
confirm). Raw CSVs: `cw_results.csv`, `decay_part1_integrator_check.csv`,
`decay_part2_atmosphere_check.csv`, `decay_part2_reference_point.csv`,
`edt_eta_results.csv`.

**Read `README.md` first**: this directory is outside R9 and outside the
byte-identical reproducibility gate; this is a one-time study, not a
continuously-maintained regression suite.

**Overclaim guard**: nothing below is "validated." Every claim is stated
as "consistent within X," "differs by X," or "reproduces to X precision,"
with an explicit statement of what the check does and does not cover.

---

## 1. CW / Hill closed form vs full nonlinear two-body (`check_cw.py`)

### What this checks and does not check

Checks ONLY the CW linearization error: the difference between (a) the
analytic CW/Hill closed-form STM (re-derived independently from the
standard textbook result -- there is exactly one correct closed form, so
independent re-derivation necessarily reproduces the same formula) and
(b) a full nonlinear point-mass two-body propagation of the same two
spacecraft from the same initial relative state. No J2, no drag, no
attitude, no control, no estimation. The nonlinear "truth" propagator is
a from-scratch `scipy.integrate.solve_ivp` (DOP853, rtol=3e-13)
integration of the point-mass two-body ODE, spot-checked against
`hapsira`'s independent `CowellPropagator` at a sample point:
**scipy-vs-hapsira position disagreement = 7.45e-6 m** at t=6000 s for a
825 km circular orbit -- i.e. the two independent integrators agree to
7 micrometers, so the "truth" propagator itself is not the source of any
of the deviations reported below.

**Caveat on case selection (read before comparing to ADSC's 0.52 m
claim):** ADSC's own ladder test (`tests/test_ladder.cpp`) cross-checks
its 14 forensic-14 pinned states (`include/adsc/forensic14_states.hpp`)
AFTER an internal abort delta-v correction (`clearing_abort_for`) that is
not reimplemented here -- doing so would require the C++ build (no local
C++ toolchain) and would not be an independent check anyway (it would
just re-derive ADSC's own code). Using forensic-14's raw pre-correction
velocities instead would produce large, physically meaningless secular
drift that is an artifact of skipping that correction, not a CW-model
error, so this script does not do that. Instead, `check_cw.py` tests
**bounded, drift-free relative-motion states** (ADSC's own
`SafetyEllipse`/`bounded_coast_min_range` family) at the same catalog
altitudes/inclinations and comparable separations (300-1700 m, spanning
and exceeding the ~350-470 m the actual forensic-14 states occupy).

### Pinned-number regression + inclination-invariance sanity checks

- `rho=300 m` corridor-hold minimum range = **424.264069 m**, matching
  `generated/reference_metrics.csv`'s `wp1_worst_coast_min_range_m` =
  424.264069 m to **2.88e-7 m** -- confirms this independent Python
  reconstruction of the WP1 geometry lands on the same pinned "424.3 m"
  number the C++ side produces.
- Inclination-invariance (pure two-body, J2 off, is spherically symmetric
  -- exactly the argument `test_ladder.cpp` itself makes): the same
  rho=300 m case at 0 deg and 51.6 deg inclination gives **identical**
  deviation to 4 decimal places (0.0354 m at orbit 1 both times),
  confirming the independent implementation respects that symmetry.

### Results table (max CW-vs-truth position deviation, meters, by orbits elapsed)

| case | alt (km) | incl (deg) | separation (m) | orbit 1 | orbit 2 | orbit 3 | orbit 4 | orbit 5 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| wp1 corridor rho=1200 | 825 | 0.0 | 1697.06 | 0.5661 | 0.5662 | 0.5664 | 0.5665 | 0.5666 |
| wp1 corridor rho=1100 | 825 | 0.0 | 1555.63 | 0.4757 | 0.4758 | 0.4759 | 0.4760 | 0.4761 |
| wp1 corridor rho=1000 | 825 | 0.0 | 1414.21 | 0.3931 | 0.3932 | 0.3933 | 0.3933 | 0.3934 |
| wp1 corridor rho=900  | 825 | 0.0 |  1272.79 | 0.3184 | 0.3185 | 0.3185 | 0.3186 | 0.3186 |
| wp1 corridor rho=800  | 825 | 0.0 |  1131.37 | 0.2516 | 0.2516 | 0.2517 | 0.2517 | 0.2517 |
| wp1 corridor rho=700  | 825 | 0.0 |   989.95 | 0.1926 | 0.1926 | 0.1927 | 0.1927 | 0.1927 |
| wp1 corridor rho=600  | 825 | 0.0 |   848.53 | 0.1415 | 0.1415 | 0.1415 | 0.1416 | 0.1416 |
| wp1 corridor rho=500  | 825 | 0.0 |   707.11 | 0.0983 | 0.0983 | 0.0983 | 0.0983 | 0.0983 |
| wp1 corridor rho=400  | 825 | 0.0 |   565.69 | 0.0629 | 0.0629 | 0.0629 | 0.0629 | 0.0629 |
| wp1 corridor rho=300 (pinned 424.3 m) | 825 | 0.0 | 424.26 | 0.0354 | 0.0354 | 0.0354 | 0.0354 | 0.0354 |
| wp1 rho=300, incl=51.6 (invariance check) | 825 | 51.6 | 424.26 | 0.0354 | 0.0354 | 0.0354 | 0.0354 | 0.0354 |
| catalog A rho=300 (840 km/71 deg) | 840 | 71.0 | 424.26 | 0.0353 | 0.0353 | 0.0353 | 0.0353 | 0.0353 |
| catalog A rho=400 | 840 | 71.0 | 565.69 | 0.0628 | 0.0628 | 0.0628 | 0.0628 | 0.0628 |
| **catalog A departure-standoff v=0 equilibrium (400 m)** | 840 | 71.0 | 400.00 | 0.4178 | 0.8357 | 1.2535 | 1.6713 | **2.0891** |
| catalog B rho=300 (750 km/78 deg) | 750 | 78.0 | 424.26 | 0.0357 | 0.0358 | 0.0358 | 0.0358 | 0.0358 |
| catalog B rho=400 | 750 | 78.0 | 565.69 | 0.0636 | 0.0636 | 0.0636 | 0.0636 | 0.0636 |
| **catalog B departure-standoff v=0 equilibrium (400 m)** | 750 | 78.0 | 400.00 | 0.4231 | 0.8462 | 1.2693 | 1.6924 | **2.1155** |

### Comparison against ADSC's own claim

ADSC's L1-ladder claim (`tests/test_ladder.cpp`, `docs/safety.md`): the
CW model's own linearization error is "measured at worst 0.52 m across
the forensic-14 states," bounded with margin by a stated 2.0 m tolerance,
over approximately one target orbital period with J2 forced off.

- The bounded drift-free ellipses at forensic-14-comparable separations
  (300-400 m at the same 840 km/71 deg and 750 km/78 deg catalog
  geometries) show a **smaller** deviation (0.035-0.064 m at orbit 1)
  than ADSC's stated 0.52 m worst case. This is not a contradiction: the
  idealized drift-null ellipse is the *minimum-nonlinearity* member of
  the family of states at a given separation (velocity chosen exactly to
  null the CW secular drift term); ADSC's actual forensic-14 states carry
  additional, effectively arbitrary post-abort radial/cross-track
  velocity components that are known (from the CW STM's own structure) to
  add further nonlinear error beyond what separation alone predicts.
  Reaching a deviation in the same range as ADSC's stated 0.52 m worst
  case required going to roughly 3-4x larger separations (1200-1700 m,
  the far end of the WP1 corridor) in the idealized drift-null family: at
  rho=1200 (1697 m separation), orbit-1 deviation is 0.5661 m --
  quantitatively close to, and consistent with, ADSC's own 0.52 m number,
  even though it comes from a different case. Empirically, deviation
  scales as separation-squared in this family (0.035382 m / (424.26 m)^2
  = 1.966e-7 m^-1, and 0.566626 m / (1697.06 m)^2 = 1.967e-7 m^-1 -- the
  same coefficient to within 0.1%), consistent with (same order
  of magnitude as) the "~3*pi*rho^2/r per orbit" scaling ADSC's own code
  comment (`tests/test_ladder.cpp`) states, though not numerically
  identical to that rough estimate (our measured coefficient is about
  3.3x smaller than the comment's `3*pi/r` prefactor evaluated at 825 km
  -- expected, since the comment is an order-of-magnitude estimate for a
  generic trajectory shape, not a precise formula for this specific
  minimum-range-at-t=0 ellipse family).
- **All 17 cases stay within the 2.0 m tolerance at orbit 1** (worst:
  0.5661 m), consistent with ADSC's claim over the ~1-orbit horizon its
  own tests use.

### First-class finding: the 2.0 m tolerance is exceeded if a departure-standoff hold is extended past ~4-5 orbits

The exact v=0 "departure-standoff" equilibrium state (400 m along-track
offset, zero relative velocity -- the same geometry
`tests/test_ladder.cpp` section 1(c) tests, and the geometry
`guidance.cpp`/`campaign.cpp` use for a post-departure hold) is an exact
fixed point of the *linear* CW model (by construction: `x0=0` makes every
secular/oscillatory term in the STM vanish, so the CW-predicted range
stays exactly 400.000 m forever) but is **not** a fixed point of the true
nonlinear two-body problem. Our independent check finds the CW-vs-truth
deviation grows **essentially linearly** with elapsed orbits: 0.4178 m
(orbit 1) -> 0.8357 -> 1.2535 -> 1.6713 -> **2.0891 m (orbit 5)** at
catalog A (840 km/71 deg), and 0.4231 -> 0.8462 -> 1.2693 -> 1.6924 ->
**2.1155 m (orbit 5)** at catalog B (750 km/78 deg) -- i.e. it **crosses
ADSC's own stated 2.0 m tolerance between orbit 4 and orbit 5** if the
hold is maintained that long.

This is **not a contradiction of any existing ADSC claim**: the 2.0 m
tolerance is explicitly exercised only over "~1 target orbital period" in
`test_ladder.cpp` (`horizon = cw.period()`), and `docs/safety.md` states
the same scope. But it does mean that scope qualifier is **load-bearing,
not merely a conservative margin**: a mission design that treats the CW
departure-standoff equilibrium as passively safe for multiple orbits
without re-verification (e.g. a long unattended hold before a later
approach) would see the CW model's own linearization error exceed the
stated tolerance by the 5th orbit.

**Mechanism (verified analytically, not just observed numerically):** a
chaser placed at a flat 400 m along-track LVLH offset from a curved
circular orbit sits about `y0^2/(2r)` off the true circular arc radially
(a standard curved-vs-tangent-line correction) -- about 1.1 cm at
r = R_Earth + 840 km. That tiny radial offset triggers the CW model's own
secular y-drift term (the `6n(sin(nt)-nt)` component of the STM), whose
per-orbit growth is `12*pi*(y0^2/(2r)) = 6*pi*y0^2/r`. Evaluating this
gives 0.4178 m/orbit for catalog A and 0.4231 m/orbit for catalog B --
matching the measured per-orbit growth rates to 4 significant figures.
So this is a real, explainable, and (given the formula) predictable
consequence of applying an exactly-flat LVLH offset to a curved orbit,
not a numerical artifact of this script's propagation.

---

## 2. Vallado exponential atmosphere vs NRLMSISE-00 (`check_decay.py`)

### Part 1 -- integrator/arithmetic check (SAME Vallado atmosphere)

Re-implements ADSC's own Vallado Table 8-4 density bands from scratch and
re-integrates the identical decay ODE with `scipy.integrate.quad`
(adaptive Gauss-Kronrod) instead of ADSC's C++ fixed-step trapezoidal
rule (6000 steps). This checks arithmetic/implementation correctness
only, NOT whether the Vallado atmosphere is a good model of reality.

| catalog | sail area (m2) | solar | quad (yr) | committed CSV (yr) | rel. err |
|---|---:|---|---:|---:|---:|
| A (SL-16, 840 km) | 100 | max | 33.6726 | 33.6726 | 2.9e-7 |
| A | 100 | min | 538.7614 | 538.7615 | 1.1e-7 |
| A | 1000 | max | 3.3673 | 3.3673 | 1.2e-5 |
| A | 1000 | min | 53.8761 | 53.8761 | 8.2e-7 |
| B (SL-8, 750 km) | 100 | max | 1.8011 | 1.8011 | 2.1e-5 |
| B | 100 | min | 28.8182 | 28.8182 | 8.6e-8 |
| B | 1000 | max | 0.1801 | 0.1801 | 7.6e-5 |
| B | 1000 | min | 2.8818 | 2.8818 | 7.0e-6 |

**Worst relative disagreement: 7.6e-5 (0.0076%).** ADSC's C++ trapezoidal
integrator (`decay.hpp`'s `integrate_decay_seconds`, 6000 steps)
reproduces an independent adaptive-quadrature reimplementation of the same
formula to well under one part in ten thousand across every committed
`generated/wp3_decay_trade.csv` decay-years cell tested. **This confirms
the C++ implementation computes the stated Vallado-atmosphere formula
correctly; it says nothing about whether that formula is realistic.**

### Part 2 -- atmosphere-MODEL check (independent NRLMSISE-00 density)

Replaces the Vallado bands with an orbit-and-local-time-averaged
NRLMSISE-00 density profile (`pymsis`, version=0), same decay ODE, same
180 km stop altitude, same 2.4 kg kit mass / Cd=2.2. The orbit average is
over 8 longitudes x 8 orbital phases (mapped to the achievable latitude
range via `sin(lat) = sin(i) sin(u)`) at a single fixed epoch -- a
documented simplification (see caveats below), not a true time-weighted
average along a real multi-year trajectory.

**Single-altitude spot check** (Vallado nominal, `solar_factor=1.0`, vs
NRLMSISE-00 at a common "moderate" solar proxy F10.7=F10.7a=150,
Ap=4, orbit-averaged at incl=51.6 deg):

| altitude | Vallado nominal | NRLMSISE-00 (F10.7=150) | ratio (Vallado/MSIS) |
|---|---:|---:|---:|
| 400 km | 3.7250e-12 kg/m3 | 2.9843e-12 kg/m3 | 1.25x |
| 750 km | 2.0563e-14 kg/m3 | 2.0252e-14 kg/m3 | 1.02x |
| 840 km | 8.4881e-15 kg/m3 | 8.4586e-15 kg/m3 | 1.00x |

At the two catalog altitudes ADSC actually decays from (750 km, 840 km),
the Vallado nominal reference and NRLMSISE-00 at a "moderate" F10.7=150
agree to within about 25% (and within ~0.3-2% at the two catalog
altitudes specifically), and the corresponding **decay-year** comparison
at that same F10.7=150 point is even closer. This table (ratio =
MSIS/Vallado) is emitted directly by `check_decay.py`'s dedicated
reference-point loop (stdout "Reference-point decay-years check..." plus
`decay_part2_reference_point.csv`; `vallado_nominal_years` =
`decay_years_vallado(..., solar_factor=1.0)`, `nrlmsise00_f107_150_years`
= `decay_years_pymsis(..., f107=150, f107a=150, ap=4.0)`):

| catalog | area (m2) | Vallado nominal (solar_factor=1, yr) | NRLMSISE-00 @ F10.7=150 (yr) | ratio (MSIS/Vallado) |
|---|---:|---:|---:|---:|
| A (840 km) | 100 | 269.3807 | 263.2860 | 0.977 |
| A (840 km) | 1000 | 26.9381 | 26.3286 | 0.977 |
| B (750 km) | 100 | 14.4091 | 14.9343 | 1.036 |
| B (750 km) | 1000 | 1.4409 | 1.4934 | 1.036 |

**These agree to within 2-4%** -- a reassuring result: at a comparable,
moderate reference solar-activity level, ADSC's Vallado exponential
density model and an independent NRLMSISE-00 evaluation give essentially
the same decay-time estimate at both catalog altitudes (the ratio is
identical between the 100 m2 and 1000 m2 sail areas, as expected --
sail area only rescales the decay ODE's time constant, it does not
change which density model is being compared).

**However**, comparing ADSC's own committed **solar_min/solar_max band**
(a flat x0.5 / x8.0 multiplier on the nominal Vallado density, applied
uniformly at every altitude, `include/adsc/mission.hpp`) against
NRLMSISE-00 evaluated at commonly-used quiet/active solar-activity
proxies (F10.7=70/Ap=4 "quiet" and F10.7=200/Ap=15 "active") shows a
real, substantial gap:

| catalog | area (m2) | ADSC solar max (yr) | NRLMSISE-00 "active" F10.7=200 (yr) | ratio | ADSC solar min (yr) | NRLMSISE-00 "quiet" F10.7=70 (yr) | ratio |
|---|---:|---:|---:|---:|---:|---:|---:|
| A | 100 | 33.6726 | 86.5678 | 2.57x | 538.7615 | 1475.5785 | 2.74x |
| A | 1000 | 3.3673 | 8.6568 | 2.57x | 53.8761 | 147.5578 | 2.74x |
| B | 100 | 1.8011 | 4.9710 | 2.76x | 28.8182 | 105.7301 | 3.67x |
| B | 1000 | 0.1801 | 0.4971 | 2.76x | 2.8818 | 10.5730 | 3.67x |

A sensitivity case at F10.7=250 ("very active," near the highest
historically-observed monthly-mean values) narrows the "solar max" gap to
1.29x (catalog A) / 1.45x (catalog B), showing the size of the gap is
sensitive to exactly which F10.7 value is treated as "solar max" -- see
`decay_part2_atmosphere_check.csv` for the full table.

### FINDING (reported, not tuned away)

At both catalog altitudes, **NRLMSISE-00 predicts a solar min-to-max
decay-time band that sits systematically 2.6x-3.7x longer than ADSC's
committed Vallado-table band**, for a commonly-used quiet/active F10.7
pairing (70/200 sfu), even though the two atmosphere models agree closely
(within a few percent) at a *shared moderate reference point* (F10.7=150,
which corresponds closely to Vallado's un-scaled nominal density at these
altitudes). The most likely explanation, given the close agreement at the
shared moderate point: ADSC's flat x0.5/x8.0 multiplicative bracket
(a deliberate, documented simplification -- `decay.hpp` already states
"a single factor is a deliberately coarse proxy for the solar cycle... the
real swing is strongly altitude-dependent") does not fully capture how
much NRLMSISE-00's actual density response to solar activity varies with
altitude in the 750-840 km band, particularly at the quiet end. The
min-to-max SPAN our NRLMSISE-00 check finds (17.0x at catalog A, 21.3x at
catalog B) is in the same range as ADSC's chosen 16x span (8.0/0.5), so
the *width* of the uncertainty band is not obviously wrong -- but its
absolute *position* (in years) may be shifted toward optimistic (shorter)
decay times relative to NRLMSISE-00, particularly at the quiet-sun end.

**What this does and does not mean:** it does NOT mean ADSC's numbers are
"wrong" -- Vallado's Table 8-4 is a well-known, deliberately coarse
textbook approximation, and NRLMSISE-00 itself has real uncertainty
(commonly cited at 10-15% in the thermosphere, larger during storms), our
orbit/local-time averaging here is a simplification (single epoch, 8x8
grid, not a true multi-year time-weighted average), and the F10.7
quiet/active proxy pairing is a choice, not a standard. But the
DIRECTION is consistent and worth flagging plainly: **if real
solar-quiet-period atmosphere is thinner than Vallado's table implies at
these altitudes (as this independent check suggests), ADSC's own
"solar min" decay-year figures may be somewhat OPTIMISTIC (i.e. real
decay could take longer than the committed table states)** -- which
matters for any sail-area sizing decision made against the 25-year IADC
guideline near the boundary. This is exactly the kind of model-choice
uncertainty `decay.hpp`'s own comments already flag qualitatively; this
check gives it a concrete, reproducible number (2.6x-3.7x at the tested
catalogs/areas) rather than leaving it as a qualitative caveat.

### STELA

**Not run.** CNES's STELA is a desktop Java Swing GUI application (the
operational French-Space-Act-reference lifetime tool), not a scriptable
library, and was out of scope for this pinned, one-time, non-interactive
Python study. What STELA would add beyond this check: (1) a real,
validated NRLMSISE-00 or JB2008 thermosphere call with an actual
historical/forecast F10.7-Ap time series (rather than this check's static
solar-activity proxies), (2) full numerical orbit propagation (including
eccentricity evolution, J2, luni-solar and SRP perturbations) rather than
the quasi-circular closed-form decay ODE both ADSC and this check use,
and (3) an independently-implemented, operationally-used decay integrator
rather than a from-scratch Python one. None of that was exercised here.

---

## 3. WP13 EDT eta(i) formula (`check_edt_eta.py`)

### What this checks

Verifies `eta_hi(i) = |cos i|` and `eta_lo(i) = cos^2(i)`
(`src/decay.cpp`'s `edt_deorbit_years`, derived in
`wp13-edt-derivation.md`) by building the full 3D aligned-dipole field
vector from first principles (not assuming the B_n=beta*cos(i) shortcut
in advance), projecting it onto an independently-constructed orbital
triad, and orbit-averaging numerically (plain numpy trapezoidal
quadrature over a dense 4001-point grid, not the derivation document's own
Gauss-Legendre-64 pseudocode) for inclinations 0 to 98.4 deg.

### Internal consistency (EMF and Lorentz-force projections agree, numerically)

For every inclination tested (0, 30, 51.6, 71, 74, 90, 98.4 deg), the
independently-computed motional EMF `(v x B).e_r` and along-track Lorentz
drag `I L (e_r x B).e_t` both reduce to the same `B_n` factor at every
sampled orbital phase to machine precision (max residual 6.9e-17 V/m /
N), and `B_n` itself is numerically confirmed u-independent (max-min over
one full orbit is 0, to within 1e-20 T of floating-point noise) --
independently reproducing the analytic claims of
`wp13-edt-derivation.md` Sections 2.3-4 by direct numerical projection
rather than by trusting the algebra.

### eta(i) numeric orbit-average vs closed form

| i (deg) | eta_F numeric | \|cos i\| closed | err | eta_SC numeric | cos^2 i closed | err |
|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 1.0000000000 | 1.0000000000 | 0 | 1.0000000000 | 1.0000000000 | 0 |
| 30.0 | 0.8660254038 | 0.8660254038 | 4.4e-16 | 0.7500000000 | 0.7500000000 | 4.4e-16 |
| 51.6 | 0.6211477803 | 0.6211477803 | 1.1e-16 | 0.3858245649 | 0.3858245649 | 1.7e-16 |
| 71.0 | 0.3255681545 | 0.3255681545 | 1.1e-16 | 0.1059946232 | 0.1059946232 | 1.4e-17 |
| 74.0 | 0.2756373558 | 0.2756373558 | 5.6e-17 | 0.0759759519 | 0.0759759519 | 4.2e-17 |
| 90.0 | 0.0000000000 | 0.0000000000 | 1.2e-32 | 0.0000000000 | 0.0000000000 | 1.4e-48 |
| 98.4 (catalog C, sun-sync) | 0.1460830286 | 0.1460830286 | 2.8e-17 | 0.0213402512 | 0.0213402512 | 6.9e-18 |

**Max |numeric - closed form| across the table: 4.4e-16 (eta_F), 4.4e-16
(eta_SC)** -- machine-precision agreement. Cross-checked separately
against `wp13-edt-derivation.md` Section 8.3's own printed
Gauss-Legendre-64 table: max disagreement 4.5e-11 (limited by that
table's own printed decimal precision, not by any discrepancy).

Limit checks (Section 6 of the derivation doc), independently
re-confirmed rather than assumed: eta(0)=1 exactly (L1); eta strictly
monotone decreasing on [0,90] deg (L2, confirmed over a dense 985-point
sweep); eta(90 deg)=0 to floating-point noise (6.1e-17, L3, the documented
aligned-dipole exact-zero artifact). A reference-radius-independence check
(eta should be a pure ratio, insensitive to the altitude used for beta)
confirms eta_F(71 deg) is identical (to 5.6e-17) whether computed at an
800 km or a 300 km reference altitude.

### Conclusion for this check

**No discrepancy found.** The closed-form eta(i) = |cos i| / cos^2(i)
claim is confirmed by an independent, from-scratch numerical
implementation to machine precision, at every inclination tested
including catalog C's 98.4 deg sun-synchronous case that the derivation
document's own printed table did not include.

**What this does NOT validate:** the aligned-dipole idealization itself
(the ~11.5 deg real dipole tilt, explicitly out of scope in both the
derivation doc and here), libration (T7, explicitly unresolved), the
absolute B0/RE_dipole constants (cited reference values, not re-derived),
or any part of the deorbit-TIME integration in `edt_deorbit_years` beyond
the eta(i) efficiency factor itself.

---

## 4. Summary

| check | result | headline number |
|---|---|---|
| CW vs full nonlinear two-body | consistent with ADSC's stated bound over ~1 orbit; **exceeds the stated 2.0 m tolerance if a departure-standoff hold is extended to ~5 orbits** (explained analytically) | worst 0.566 m @ orbit 1 (17 cases); 2.09-2.12 m @ orbit 5 (2 cases) |
| Decay integrator (same atmosphere) | consistent to within numerical noise | worst rel. err 7.6e-5 |
| Decay atmosphere model (Vallado vs NRLMSISE-00) | **real, reported discrepancy**: NRLMSISE-00 gives systematically longer decay times at the tested quiet/active F10.7 proxies, though the two models agree closely at a shared moderate reference point | 2.57x-3.67x band-edge gap; 0.98-1.04x at the shared moderate point |
| EDT eta(i) formula | fully confirmed, no discrepancy | max err 4.4e-16 vs closed form |

Two genuine, non-tuned-away findings came out of this study (the
departure-standoff multi-orbit tolerance crossing, and the Vallado-vs-
NRLMSISE-00 decay-band gap); one check (EDT eta) found full agreement to
machine precision. Both findings are explained mechanistically above, not
just reported as raw numbers, and neither contradicts an existing ADSC
claim outright -- both sharpen the *scope* of an existing claim (the
2.0 m tolerance's implicit ~1-orbit horizon; the decay model's own
documented "coarse solar-cycle proxy" caveat) with a concrete, reproducible
number.
