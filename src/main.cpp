#include "ADSC_Core.h"
#include <iostream>

int main() {
    ADSC_Core core;
    std::cout << "=== ADSC v1.21 Reality Final Edition Start ===\n";

    Eigen::Vector3d r_attach(0, 0, 0.15);
    Eigen::Vector3d r_rel(0.08, 0.04, 0.11);
    Eigen::Vector3d v_ok(0.12, 0.09, 0.10);   // 0.15 m/s以内

    core.post_capture_stabilization(true, 2.4, r_attach, r_rel, v_ok);
    core.deorbit_protocol(true);
    core.deorbit_protocol(false);

    std::cout << "=== v1.21 Reality Final Simulation Complete ===\n";
    return 0;
}
