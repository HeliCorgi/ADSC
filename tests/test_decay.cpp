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

    std::printf("decay: all tests passed\n");
    return 0;
}
