// WP3 decay-physics tests: the integrator against its constant-density
// closed form, atmosphere/decay monotonicity, the honest sail-area structure,
// and the EDT knob. Explicit return-1 checks (R4), fixed constants, no RNG (R6).
#include <cmath>
#include <cstdio>

#include "adsc/decay.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

int main() {
    // 1. Integrator vs the constant-density closed form
    //    2(sqrt(a0) - sqrt(a)) = rho*Cd*(A/m)*sqrt(mu) * t.
    {
        const double rho = 1.0e-13, cd = 2.2, a_over_m = 0.01;
        const double a0 = kEarthRadius + 800.0e3;
        const double a_stop = kEarthRadius + 200.0e3;
        const double t_num = integrate_decay_seconds(
            a0, a_stop, cd, a_over_m, [rho](double) { return rho; });
        const double t_closed = 2.0 * (std::sqrt(a0) - std::sqrt(a_stop)) /
                                (rho * cd * a_over_m * std::sqrt(kEarthMu));
        CHECK(std::abs(t_num - t_closed) / t_closed < 1e-4);
    }

    // 2. Atmosphere: strictly decreasing with altitude; exactly linear in the
    //    solar factor.
    {
        CHECK(atmospheric_density(300e3, 1.0) > atmospheric_density(800e3, 1.0));
        CHECK(atmospheric_density(700e3, 1.0) > atmospheric_density(750e3, 1.0));
        CHECK(atmospheric_density(800e3, 1.0) > atmospheric_density(1000e3, 1.0));
        CHECK(std::abs(atmospheric_density(500e3, 2.0) -
                       2.0 * atmospheric_density(500e3, 1.0)) < 1e-20);
    }

    // 3. Decay monotonicity: bigger sail, lower target, and higher solar
    //    density each shorten the decay.
    {
        const DebrisCatalog A = catalog_A(), B = catalog_B();
        const double kit = 2.4, cd = 2.2, stop = 180.0, smin = 0.5, smax = 8.0;
        CHECK(sail_decay_years(A, 50.0, kit, cd, stop, smin) >
              sail_decay_years(A, 200.0, kit, cd, stop, smin));
        CHECK(sail_decay_years(A, 100.0, kit, cd, stop, smin) >
              sail_decay_years(A, 100.0, kit, cd, stop, smax));
        CHECK(sail_decay_years(A, 100.0, kit, cd, stop, smin) >
              sail_decay_years(B, 100.0, kit, cd, stop, smin));
    }

    // 4. Honest structure (the WP3 deliverable): a multi-tonne stage at ~840 km
    //    needs an impractically large sail for the 25-year guideline, far more
    //    than the lighter/lower catalog B -> this brackets the sail/EDT
    //    crossover (T1). The numbers are printed for the evidence pack.
    {
        const double kit = 2.4, cd = 2.2, stop = 180.0, smin = 0.5;
        const double areaA = area_for_target_years(catalog_A(), 25.0, kit, cd, stop, smin);
        const double areaB = area_for_target_years(catalog_B(), 25.0, kit, cd, stop, smin);
        std::printf("decay: 25-yr sail area (solar min) A=%.0f m^2, B=%.0f m^2\n",
                    areaA, areaB);
        CHECK(areaA > 100.0);   // sail-only on catalog A is impractical
        CHECK(areaA > areaB);   // A needs far more area than B (T1 crossover)

        const double yrsA = sail_decay_years(catalog_A(), 120.0, kit, cd, stop, smin);
        std::printf("decay: catalog A 120 m^2 sail (solar min) = %.1f yr (>25 guideline)\n",
                    yrsA);
        CHECK(yrsA > 25.0);     // the honest negative that motivates the EDT branch
    }

    // 5. area_for_target_years round-trips exactly through sail_decay_years.
    {
        const double kit = 2.4, cd = 2.2, stop = 180.0, sf = 3.0;
        const double area = area_for_target_years(catalog_B(), 25.0, kit, cd, stop, sf);
        const double yrs = sail_decay_years(catalog_B(), area, kit, cd, stop, sf);
        CHECK(std::abs(yrs - 25.0) < 1e-6);
    }

    // 6. EDT parametric knob: exact and monotone in the decay-rate parameter.
    {
        const double a0 = kEarthRadius + 840e3, a_stop = kEarthRadius + 180e3;
        const double d1 = edt_deorbit_days(a0, a_stop, 50.0);
        const double d2 = edt_deorbit_days(a0, a_stop, 100.0);
        CHECK(std::abs(d1 - (a0 - a_stop) / 50.0) < 1e-6);
        CHECK(d2 < d1);
    }

    // 7. EDT physics core (WP13): eta(i) = |cos i| (optimistic) / cos(i)^2
    //    (conservative), eta(0) = 1, matching the derivation-note table
    //    (wp13-edt-derivation.md Section 8.3).
    {
        const DebrisCatalog equatorial{"eta-test equatorial", 9000.0, 840.0, 0.0, false};
        const EdtConfig cfg{};
        const EdtResult eq = edt_deorbit_years(equatorial, cfg, 180.0);
        CHECK(std::abs(eq.eta_hi - 1.0) < 1e-12);
        CHECK(std::abs(eq.eta_lo - 1.0) < 1e-12);

        const EdtResult a71 = edt_deorbit_years(catalog_A(), cfg, 180.0);
        CHECK(std::abs(a71.eta_hi - 0.3255681545) < 1e-6);
        CHECK(std::abs(a71.eta_lo - 0.1059946232) < 1e-6);
    }

    // 8. EDT deorbit-time band: the conservative (EMF-limited, cos^2 i) edge
    //    always takes longer than the optimistic (power-capped, cos i) edge
    //    for a non-equatorial catalog -- never collapse to a point value.
    {
        const EdtResult a = edt_deorbit_years(catalog_A(), EdtConfig{}, 180.0);
        CHECK(std::isfinite(a.years_optimistic));
        CHECK(std::isfinite(a.years_conservative));
        CHECK(a.years_conservative > a.years_optimistic);
    }

    // 9. Cross-check: the public power-law EDT integrator against the
    //    elementary closed form t = m*sqrt(mu)*(a0^2.5 - as^2.5) / (5*C) for
    //    F(a) = C / a^3 (aligned-dipole B_n ~ a^-3, wp13-edt-derivation.md
    //    Section 8.1), and against edt_deorbit_years' own optimistic-edge
    //    output for the SAME physical constants (verifies the algebra AND the
    //    wiring, mirroring CHECK 1's integrator-vs-closed-form pattern).
    {
        const double kB0_T = 3.01153e-5;             // SPENVIS centred dipole, IGRF 2000
        const double kDipoleRefRadius = 6371200.0;    // m, SPENVIS centred-dipole radius
        const double kSecPerYear = 365.25 * 86400.0;

        const DebrisCatalog A = catalog_A();
        const EdtConfig cfg{};
        const double cos_i = std::cos(A.inclination_deg * kPi / 180.0);
        const double a0 = kEarthRadius + A.altitude_km * 1000.0;
        const double a_stop = kEarthRadius + 180.0 * 1000.0;
        const double m = A.mass_kg + cfg.kit_mass_kg;
        const double C_hi = cfg.avg_current_a * cfg.tether_length_m * kB0_T *
                            std::pow(kDipoleRefRadius, 3.0) * cos_i * cfg.eta_libration;

        const double t_num = integrate_edt_seconds(
            a0, a_stop, m, [C_hi](double a) { return C_hi / (a * a * a); });
        const double t_closed = m * std::sqrt(kEarthMu) *
                                (std::pow(a0, 2.5) - std::pow(a_stop, 2.5)) / (5.0 * C_hi);
        CHECK(std::abs(t_num - t_closed) / t_closed < 1e-6);

        const EdtResult r = edt_deorbit_years(A, cfg, 180.0);
        const double closed_years = t_closed / kSecPerYear;
        CHECK(std::abs(r.years_optimistic - closed_years) / closed_years < 1e-6);
    }

    // 10. Monotonicity: years increase (deorbit is slower) as inclination
    //     departs from equatorial, matching eta(i)'s monotonic decrease.
    {
        const DebrisCatalog A = catalog_A();
        DebrisCatalog A_equatorial = A;
        A_equatorial.inclination_deg = 0.0;
        const EdtConfig cfg{};
        const EdtResult incl = edt_deorbit_years(A, cfg, 180.0);
        const EdtResult equ = edt_deorbit_years(A_equatorial, cfg, 180.0);
        CHECK(incl.years_optimistic > equ.years_optimistic);
        CHECK(incl.years_conservative > equ.years_conservative);
    }

    // 11. Polar orbit: the aligned-dipole orbit-normal field is exactly zero
    //     (wp13-edt-derivation.md Section 6, limit L3) -> infinite deorbit
    //     time, reported honestly rather than as a divide-by-zero artifact.
    {
        const DebrisCatalog polar{"eta-test polar", 9000.0, 840.0, 90.0, false};
        const EdtResult p = edt_deorbit_years(polar, EdtConfig{}, 180.0);
        CHECK(std::isinf(p.years_optimistic));
        CHECK(std::isinf(p.years_conservative));
    }

    // 12. Retrograde orbit (catalog_C, Envisat-class at 98.4 deg): the passive
    //     tether's induced current always drags (Lenz), so the force magnitude
    //     scales with |cos i| and the deorbit time is FINITE -- only an
    //     exactly-polar orbit nulls the aligned-dipole coupling. eta_hi is the
    //     magnitude |cos 98.4 deg|, eta_lo its square.
    {
        const EdtResult r = edt_deorbit_years(catalog_C(), EdtConfig{}, 180.0);
        CHECK(std::abs(r.eta_hi - 0.1460830286) < 1e-6);
        CHECK(std::abs(r.eta_lo - 0.0213402512) < 1e-6);
        CHECK(!std::isinf(r.years_optimistic));
        CHECK(!std::isinf(r.years_conservative));
        CHECK(r.years_conservative > r.years_optimistic);
        CHECK(r.emf_v > 0.0);  // reported as a magnitude (polarity note in docs)
    }

    std::printf("decay: all tests passed\n");
    return 0;
}
