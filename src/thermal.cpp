#include "adsc/thermal.hpp"

namespace adsc {

bool ThermalPCM::absorb(double power_watts, double dt) {
    elapsed_   += dt;
    remaining_ -= power_watts * dt;
    if (remaining_ < 0.0) remaining_ = 0.0;
    return saturated();
}

}  // namespace adsc
