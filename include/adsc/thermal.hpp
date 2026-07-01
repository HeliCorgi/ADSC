#pragma once

namespace adsc {

// Passive phase-change-material (PCM) heat sink. A single lumped bucket: absorbs
// waste heat until the latent-heat budget is exhausted, at which point the bus
// must enter safe mode. This is a first-order energy balance only — no
// eclipse/sunlight radiative model, no per-node conduction.
class ThermalPCM {
public:
    ThermalPCM(double capacity_joules, double max_time_seconds)
        : capacity_(capacity_joules),
          remaining_(capacity_joules),
          max_time_(max_time_seconds) {}

    // Absorb heat at a given power over dt. Returns true once saturated (either
    // the PCM budget is spent or the safe-operation time limit is reached).
    bool absorb(double power_watts, double dt);

    bool   saturated()      const { return remaining_ <= 0.0 || elapsed_ > max_time_; }
    double remaining()      const { return remaining_; }
    double fraction_left()  const { return remaining_ / capacity_; }
    double elapsed()        const { return elapsed_; }

private:
    double capacity_;
    double remaining_;
    double max_time_;
    double elapsed_ = 0.0;
};

}  // namespace adsc
