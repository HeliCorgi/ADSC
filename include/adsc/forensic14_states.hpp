#pragma once

// WP11 forensic-14 pinned states (R15), SHARED between src/main_ladder.cpp
// and tests/test_ladder.cpp (WP12). PROVENANCE: this table is a verbatim
// duplicate of tests/test_forensic14.cpp's kForensic14 (itself copied from
// generated/wp10_violation_forensics.csv columns r0_x_m, r0_y_m, r0_z_m,
// v0_x_m_per_s, v0_y_m_per_s, v0_z_m_per_s, WP10c committed-generated
// values). Duplicated rather than shared via a refactor of
// tests/test_forensic14.cpp so that already-regression-tested file is left
// completely untouched (R1); if the forensics ever regenerate, BOTH copies
// must be updated together (R15 re-baselining protocol) -- same pin status
// as the 19.15 / 424.3 / 16.87 / 17.07 s reference numbers.

namespace adsc {

struct Forensic14State {
    char   catalog;  // 'A' = SL-16 / Zenit-2 (840 km), 'B' = SL-8 / Kosmos-3M (750 km)
    int    run_index;
    double x0, y0, z0, vx0, vy0, vz0;
};

// C++17 inline variable: one shared definition across every translation unit
// that includes this header (no ODR risk from the two consumers above).
inline constexpr Forensic14State kForensic14States[] = {
    {'A', 65,  -109.2047, -390.9569,  47.6195,  0.053575,  0.002560, -0.004322},
    {'A', 81,   -62.6103, -295.8113,   3.2022, -0.015877, -0.010833,  0.008057},
    {'A', 86,    88.9285, -377.7154, -36.4953, -0.033824,  0.021650, -0.010427},
    {'A', 112,  103.1442, -403.3742, -17.1617, -0.005783, -0.001262, -0.006628},
    {'A', 340,  142.3706, -444.1451,  39.5522,  0.014951, -0.009629,  0.020602},
    {'A', 460,  -80.9837, -340.0491, -30.4610, -0.019190, -0.032109,  0.028635},
    {'A', 490, -136.5058, -403.0482,  31.5876,  0.000136, -0.023902, -0.020074},
    {'B', 88,  -110.7634, -340.9315, -23.5717,  0.024717,  0.019986, -0.001310},
    {'B', 99,   -78.3053, -281.4154,  -6.6638, -0.015060, -0.001444, -0.005980},
    {'B', 160, -126.4040, -380.8614,  20.7493, -0.005108,  0.007905,  0.005312},
    {'B', 254, -136.6149, -326.4996, -67.5308, -0.014607,  0.003844, -0.002598},
    {'B', 362,  -88.5700, -376.2516, 125.0294,  0.021529,  0.000352,  0.033527},
    {'B', 383,  -81.6652, -346.3719,  39.5522, -0.010185,  0.011825,  0.016236},
    {'B', 419,  107.7674, -403.6285,  85.6915, -0.013524,  0.015787, -0.004265},
};
inline constexpr int kForensic14Count =
    static_cast<int>(sizeof(kForensic14States) / sizeof(kForensic14States[0]));

}  // namespace adsc
