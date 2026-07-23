# WP13 Annex — Orbit-Averaged Inclination Efficiency eta(i) for a Nadir-Aligned Electrodynamic Tether

Status: derivation for review (Fable5 reviews before coding, WP13 task-2).
Scope: closed-form derivation + deterministic quadrature cross-check of the
mandatory inclination-dependent v x B efficiency (spec:247-251, instr 0.1).
All math ASCII; every step reproducible. Numerics cross-checked by a 64-node
Gauss-Legendre quadrature (script at end; results pinned in Section 8).

--------------------------------------------------------------------------
## 0. Result up front (for the impatient reviewer)

For a tether deployed along the local vertical (radial / nadir-zenith) in an
Earth-centered ALIGNED-dipole field, on a circular orbit of radius r and
inclination i, the orbit-averaged along-track (drag) efficiency is CLOSED FORM:

    Fixed-current model (I held at a power/limit-capped value):
        eta_F(i)  = cos(i)

    Self-consistent model (current scales with the motional EMF, I ~ EMF):
        eta_SC(i) = cos(i)^2

both normalized so eta(0) = 1. No quadrature is actually required for the
aligned dipole; the 64-node Gauss-Legendre integration in Section 8 reproduces
these to 2.2e-16 and exists only as a cross-check and as the general-field
harness (a future tilted dipole reintroduces u-dependence).

Why so clean: for a radial tether the ONLY field component that produces an
along-track Lorentz force -- and the only one that appears in the radial
tether's motional EMF -- is the orbit-normal component B_n, and for an aligned
dipole B_n = B0 (RE/r)^3 cos(i) is CONSTANT around the orbit. The u-varying
components B_r and B_t cancel out of the along-track projection.

--------------------------------------------------------------------------
## 1. Assumptions and caveats (stated, not hidden)

A1. Circular orbit, radius r, inclination i, argument of latitude u measured
    from the ascending node. Orbital speed v = sqrt(GM/r), inertial.
A2. Tether along the local vertical: t_hat = e_r (radial, zenith-nadir).
    Rigid, straight; libration IGNORED here (T7 open -- separate penalty knob,
    instr 0.2; never claimed solved).
A3. Geomagnetic field = Earth-centered ALIGNED dipole: moment along the Earth
    rotation / geographic axis z_hat.
    CAVEAT (tilt): the real dipole is tilted ~11.5 deg (and offset). The tilt
    rotates B_n by a slowly varying phase and, critically, makes eta(90 deg)
    small-but-nonzero instead of exactly zero (Section 6). Treat the aligned
    result as the leading-order geometry; carry tilt as an uncertainty, not a
    correction baked into v1.
A4. Non-rotating plasma / inertial frame for the motional EMF: the corotation
    electric field E_corot = -(Omega_E x r) x B is IGNORED.
    CAVEAT (corotation): at LEO v_orbit ~ 7.6 km/s while the corotation speed
    ~0.4 km/s, so the neglected term is ~5% of the EMF and is largest near the
    equator; it does not change the cos(i) geometry, only the scalar prefactor.
    Fold into the current-efficiency band, do not model explicitly in v1.
A5. B0 = mu0 m /(4 pi RE^3) ~ 3.12e-5 T is the equatorial surface field.
    Define the shorthand  beta(r) = B0 (RE/r)^3.

--------------------------------------------------------------------------
## 2. Aligned-dipole field in the orbital frame

### 2.1 Field vector form
A magnetic dipole of moment m_vec = m * m_hat produces, at position R,

    B = (mu0 m / 4 pi R^3) [ 3 (m_hat . r_hat) r_hat - m_hat ].

For Earth the external field points NORTH at the equator, which requires the
moment to point SOUTH: m_hat = -z_hat. With beta = B0 (RE/r)^3,

    B = beta [ 3 (-z_hat . r_hat) r_hat - (-z_hat) ]
      = beta [ z_hat - 3 (z_hat . r_hat) r_hat ].            (2.1)

Sanity: at the equator z_hat . r_hat = 0 => B = beta z_hat (northward). OK.

### 2.2 Orbital frame unit vectors
Place the ascending node on the x-axis (RAAN = 0; RAAN only rotates the whole
picture about z and drops out of orbit averages). At argument of latitude u:

    r_hat = ( cos u ,  sin u cos i ,  sin u sin i )           (radial out)
    e_t   = d r_hat/du = ( -sin u , cos u cos i , cos u sin i ) (along-track)
    e_n   = r_hat x e_t = ( 0 , -sin i , cos i )               (orbit normal)

e_n is constant (fixed orbit plane), as it must be. (e_r, e_t, e_n) is a
right-handed triad: e_r x e_t = e_n, e_t x e_n = e_r, e_n x e_r = e_t.

The z-projections we will need:
    z_hat . r_hat = sin i sin u        (= cos(colatitude) = sin(mag. latitude))
    z_hat . e_t   = sin i cos u
    z_hat . e_n   = cos i

Note the magnetic latitude relation sin(lambda) = sin i sin u (spherical
triangle: a great circle inclined at i has latitude lambda at angle u from the
node with sin lambda = sin i sin u).

### 2.3 Components (project 2.1 onto the triad)
Using B . a = beta [ (z_hat.a) - 3 (z_hat.r_hat)(r_hat.a) ] and r_hat.e_t =
r_hat.e_n = 0, r_hat.r_hat = 1:

    B_r = B . e_r = beta [ sin i sin u - 3 sin i sin u ] = -2 beta sin i sin u
    B_t = B . e_t = beta [ sin i cos u - 0 ]             =    beta sin i cos u
    B_n = B . e_n = beta [ cos i - 0 ]                   =    beta cos i        (2.3)

    B_r = -2 B0 (RE/r)^3 sin i sin u   (radial)
    B_t =    B0 (RE/r)^3 sin i cos u   (along-track)
    B_n =    B0 (RE/r)^3 cos i         (orbit-normal, CONSTANT in u)

The B_r form matches the value given in the task statement.

### 2.4 Consistency check vs textbook spherical components
Geomagnetic spherical components (theta = colatitude): B_r = -2 beta cos theta,
B_theta = -beta sin theta, B_phi = 0. With cos theta = sin i sin u this gives
B_r = -2 beta sin i sin u (matches 2.3). The tangential-plane magnitude must be
frame-independent:

    B_theta^2 + B_phi^2 = beta^2 sin^2 theta = beta^2 (1 - sin^2 i sin^2 u)
    B_t^2 + B_n^2 = beta^2 (sin^2 i cos^2 u + cos^2 i) = beta^2 (1 - sin^2 i sin^2 u)

They agree identically. Full magnitude: |B| = beta sqrt(1 + 3 sin^2 i sin^2 u)
= beta sqrt(1 + 3 sin^2 lambda). OK.

--------------------------------------------------------------------------
## 3. (a) Motional EMF along the radial tether

Per unit length the motional field is (v x B); projected on the tether
direction t_hat = e_r it drives the current:

    E_m(u) = (v x B) . e_r ,   v = v e_t .

Compute v x B with v = v e_t and B = B_r e_r + B_t e_t + B_n e_n:

    v x B = v [ B_r (e_t x e_r) + B_n (e_t x e_n) ]
          = v [ -B_r e_n + B_n e_r ]                         (2.2 triad rules)

Projection on e_r:

    E_m(u) = v B_n = v beta cos i = v B0 (RE/r)^3 cos i.       (3.1)

KEY: the radial-tether EMF sees ONLY B_n. It is constant around the orbit and
scales as cos i. The open-circuit tether voltage is V_emf = E_m * L =
v B0 (RE/r)^3 cos i * L.

--------------------------------------------------------------------------
## 4. (b) Lorentz drag force for radial current

Current I flows along the tether, so the current element is I L e_r. The force

    F = I L (e_r x B)
      = I L [ B_t (e_r x e_t) + B_n (e_r x e_n) ]
      = I L [ B_t e_n - B_n e_t ].                            (4.1)

Two components:
  - Along-track (drag / energy):   F_t = F . e_t = -I L B_n = -I L beta cos i
  - Orbit-normal (out-of-plane):   F_n = F . e_t? no:  F_n = F . e_n = I L B_t
                                        = I L beta sin i cos u

The sign of I is fixed by the passive EMF-driven current so that F_t opposes v
(drag). The DRAG magnitude that removes orbital energy is

    |F_t| = I L B_n = I L B0 (RE/r)^3 cos i.                  (4.2)

The out-of-plane F_n = I L beta sin i cos u averages to zero over an orbit and,
being perpendicular to v, does ZERO work: it perturbs node/inclination at
second order but does NOT drive secular semimajor-axis decay. Only F_t enters
the deorbit rate (Gauss planetary eq.: da/dt = (2/n) * (F_t / m) for a
circular orbit, i.e. da/dt is proportional to the along-track force).

--------------------------------------------------------------------------
## 5. (c) Same geometry factor; definition of eta(i)

From (3.1) and (4.2), BOTH the EMF projection and the along-track force
projection carry the identical field-geometry factor

    B_n = beta cos i     (the orbit-normal field, constant over u).

This is the whole content of the "inclination-dependent v x B efficiency": a
radial tether couples to the orbit-normal field only, and for an aligned dipole
that projection is cos i.

Define eta(i) = (orbit-averaged secular drag at i) / (same at i = 0), so
eta(0) = 1. Let < . > = (1/2pi) integral_0^{2pi} ( . ) du.

### 5.1 Model (i): FIXED current (force-averaged)
I is set by the power budget / hardware current cap and held constant vs i.
Secular drag ~ < |F_t| > = I L beta < |cos i| >. Since B_n is constant and
single-signed over the orbit (no cancellation),

    eta_F(i) = < |B_n(u,i)| > / < |B_n(u,0)| > = |cos i| / 1 = cos i.   (5.1)

### 5.2 Model (ii): SELF-CONSISTENT current (I ~ EMF)
When the current is EMF/collection-limited rather than power-capped, I(u) is
proportional to the local EMF. In the linear (resistively-limited or
short-circuit) regime I ~ E_m ~ v B_n. The secular drag is then the orbit
average of the PRODUCT (EMF factor) x (force factor):

    F_t ~ I * B_n ~ B_n^2 ,   so
    eta_SC(i) = < B_n(u,i)^2 > / < B_n(u,0)^2 > = cos^2 i.              (5.2)

General statement (holds for a future u-dependent B_n, e.g. tilted dipole):

    eta_F(i)  = < |B_n(u,i)| >            / < |B_n(u,0)| >
    eta_SC(i) = < EMF(u,i) |B_n(u,i)| >   / < EMF(u,0) |B_n(u,0)| >
              = < B_n(u,i)^2 > / < B_n(u,0)^2 >     (EMF ~ B_n).

For the aligned dipole B_n is u-independent and these collapse to cos i and
cos^2 i respectively. A nonlinear OML law I ~ EMF^(3/2) would give an
intermediate power ~ cos^(1+3/2)=cos^2.5 i locally, but bracketed by the two
models above; cos i and cos^2 i are the honest optimistic/conservative edges.

--------------------------------------------------------------------------
## 6. Limit checks

L1. eta(0) = 1 for both (equatorial orbit: orbit normal = z_hat = north, the
    full dipole equatorial field is orbit-normal => maximal coupling). PASS.

L2. Monotone decreasing on [0, 90 deg]: cos i and cos^2 i are both strictly
    decreasing there. PASS (verified numerically, Section 8).

L3. eta(90 deg): EXACTLY ZERO for the aligned dipole (both models):
    eta_F(90) = cos 90 = 0, eta_SC(90) = 0.
    WHY (physical, not a bug): the aligned-dipole field always lies in the
    meridian plane spanned by z_hat and r_hat, i.e. B . e_n = beta[ z_hat.e_n
    - 3(z_hat.r_hat)(r_hat.e_n) ]. For a polar orbit the orbit plane CONTAINS
    z_hat, so its normal e_n is equatorial (z_hat.e_n = cos 90 = 0) and r_hat
    lies in a plane orthogonal to e_n (r_hat.e_n = 0). Hence B_n = 0 everywhere
    on a polar orbit, so a radial current has zero along-track force at every
    point -- not just on average. A polar radial tether does no secular drag in
    an aligned dipole. The out-of-plane F_n = I L beta sin i cos u is nonzero at
    90 deg but does no work.
    HONESTY NOTE: this exact zero is an ARTIFACT of the aligned-dipole + radial-
    tether idealization. The ~11.5 deg dipole tilt tips e_n out of the field's
    null, restoring a small nonzero drag of order (sin 11.5 deg)^2 ~ 0.04 of the
    equatorial value (order-of-magnitude; a tilted-dipole quadrature would pin
    it). So report eta(90) = 0 for the aligned model and flag "true value is
    small-but-nonzero, dominated by dipole tilt" as the caveat. Neither catalog
    is at 90 deg (A ~71, B ~74), so this does not touch the headline numbers.

--------------------------------------------------------------------------
## 7. Recommendation for ADSC v1

Expectation under test (from the task): "fixed-average-current with a power-
budget cap is the simplest defensible model" => eta = cos i. Verdict: JUSTIFIED
as the OPTIMISTIC edge, but it must NOT be shipped as a point value.

Reasoning:
 - eta_F = cos i is exactly correct WHEN the binding limit is the power supply /
   hardware max current, so I really is inclination-independent. It is the
   simplest, and it is the honest UPPER efficiency.
 - eta_SC = cos^2 i is exactly correct WHEN the current is EMF / OML-collection
   limited (I falls with the EMF, which itself falls as cos i). It is the honest
   LOWER efficiency. At 71 deg it is 0.106 vs 0.326 -- a ~3x heavier penalty.
 - A real bare tether at ~800 km, 71-74 deg with realistic Ne sits BETWEEN
   these regimes and can switch between them over solar min/max. Committing to
   one point value would either overstate EDT at high i (cos i) -- exactly the
   trap the WP13 claims-audit exists to catch (instr 3) -- or understate it.

RECOMMENDATION: implement BOTH as the inclination contribution to the MANDATORY
uncertainty band (spec: "never a point value"):

    eta_hi(i) = cos i      (optimistic edge; power/limit-capped current)
    eta_lo(i) = cos^2 i    (conservative edge; EMF/collection-limited current)

and report the deorbit-time band with eta in [eta_lo, eta_hi]. The band widens
at high inclination precisely where honesty matters most, and the conservative
edge structurally prevents EDT from silently "fixing" class A at 71 deg.

Cleanest single-code realization (avoids double counting): apply the geometry
factor cos i ONCE to the EMF and ONCE to the force, and let the current model
decide the regime -- if I is power-capped the net inclination scaling is cos i;
if I is computed from the EMF the net scaling is cos^2 i automatically. Then the
band above is just the two current-limit regimes, not an ad hoc fudge.

--------------------------------------------------------------------------
## 8. C++-implementable formula + deterministic quadrature cross-check

### 8.1 What v1 actually computes (O(1), closed form)
```
// inputs: i [rad], r [m]; constants B0=3.12e-5 T, RE=6.371e6 m, GM=3.986004418e14
double beta = B0 * pow(RE/r, 3.0);        // T
double v    = sqrt(GM / r);               // m/s
double Bn   = beta * cos(i);              // orbit-normal field (constant in u)
// motional EMF per unit length and open-circuit voltage:
double Em   = v * Bn;                      // V/m
double Vemf = Em * L;                      // V   (L = tether length)
// inclination efficiency knobs (eta(0)=1):
double eta_hi = cos(i);                    // fixed / power-capped current
double eta_lo = cos(i)*cos(i);             // self-consistent EMF-limited current
// along-track drag magnitude: |F_t| = I * L * Bn   (I from the current model)
```
No branch, no iteration, pure double, no external deps. cos i / cos^2 i are the
efficiency multipliers on the force.

### 8.2 Fixed 64-node Gauss-Legendre harness (general field, deterministic)
Needed only if B_n acquires u-dependence (tilted dipole extension). Nodes are
generated by Newton-Raphson on the Legendre polynomial P_64 at init -- fully
deterministic, SEEDLESS, no RNG, no external table. Map GL nodes x_k in [-1,1]
to u_k = pi (x_k + 1) in [0, 2pi); orbit average = 0.5 * sum_k w_k f(u_k).

    eta_F(i)  = avg_u |Bn(u,i)|      / avg_u |Bn(u,0)|
    eta_SC(i) = avg_u  Bn(u,i)^2     / avg_u  Bn(u,0)^2

with Bn(u,i) = beta cos i for the aligned dipole (u-independent => the average
is exact for any node count; N=64 chosen for the general case). Pseudocode for
node generation:
```
for k in 0..N-1:
  z = cos(pi*(k+0.75)/(N+0.5));                 // initial guess
  repeat:
    p0=1; p1=0;
    for j in 0..N-1: p2=p1; p1=p0; p0=((2j+1)*z*p1 - j*p2)/(j+1);  // P_N(z)
    dp = N*(z*p0 - p1)/(z*z - 1);               // P_N'(z)
    dz = p0/dp; z -= dz;
  until |dz| < 1e-15;
  x[k]=z; w[k]=2/((1-z*z)*dp*dp);
```

### 8.3 Verified numeric table (GL-64 quadrature == closed form to 2.2e-16)
Catalog inclinations: A 71 deg, B 78 deg (the code presets; note the real
SL-8 population clusters at 74/83 deg -- wp13-literature.md Topic 5b -- so
78 is a PLACEHOLDER compromise between the clusters).

    i (deg) | eta_F = cos i | eta_SC = cos^2 i
    --------+---------------+-----------------
      0.0   | 1.0000000000  | 1.0000000000
     30.0   | 0.8660254038  | 0.7500000000
     51.6   | 0.6211477803  | 0.3858245649
     71.0   | 0.3255681545  | 0.1059946232
     74.0   | 0.2756373558  | 0.0759759519
     90.0   | 0.0000000000  | 0.0000000000

Limit checks (numerically confirmed):
  - eta_F(0)=1, eta_SC(0)=1                          PASS (L1)
  - strictly monotone decreasing 0->90 deg           PASS (L2)
  - eta_F(90)=6.1e-17, eta_SC(90)=3.7e-33 (== 0)      PASS (L3, exact-zero limit)
  - max | GL-64 - closed form | over 10 sample i      = 2.2e-16

### 8.4 Reproduce
Python cross-check script (`/tmp/wp13_eta.py` at authoring time; portable):
implements the GL-64 harness above and the closed forms, printing the table and
all four limit checks. Deterministic, no seeds, no external deps.

--------------------------------------------------------------------------
## 9. One-line summary for the evidence pack
Radial EDT drag couples only to the orbit-normal field B_n = B0 (RE/r)^3 cos i;
orbit-averaged inclination efficiency is eta = cos i (fixed/power-capped current)
to cos^2 i (EMF/collection-limited current), = 0.33..0.11 at 71 deg -- ship the
[cos^2 i, cos i] band, never a point value. eta(90 deg)=0 in the aligned dipole
(tilt makes the true value small-but-nonzero). Libration (T7) remains open.

--------------------------------------------------------------------------
## 10. WP16 follow-up (2026-07)
A lumped-mass multi-bead reimplementation of the same aligned-dipole B_n
geometry now exists (owner-directed extension, not a spec work package),
together with in-model controller results (constant current, phase-gated,
fixed-duty) and a twin-to-twin state/parameter-estimation demo: see
docs/digital_twin.md and generated/wp16_twin.md (data: generated/wp16_twin.csv,
schema: generated/wp16_twin_schema.md). Dumbbell-limit validation there
reproduces this study's own eps-based bounded/tumble classification (Section 5
above via _tasks_local/t7-libration-study.md Sec 5.1), with a stated
model-family/integrator offset at the bounded case, never a claimed exact
match. eta_libration remains exactly what Section 0/9 above already say it
is: a bookkeeping duty-cycle knob folded flat into the edt_years band, NOT
validated by WP16 as a stability mechanism -- if anything, WP16's own
Monte Carlo finds naive fixed-duty cycling can excite libration in-model,
which sharpens rather than resolves the caution already on record here. T7
(libration / dynamic tether stability) is UNCHANGED by WP16: it remains
open, and no controller in either study is a resolved stability mechanism.
