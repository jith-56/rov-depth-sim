#include "rovsim/safety_monitor.hpp"

namespace rovsim {

const char* to_string(SafetyState s) {
    switch (s) {
        case SafetyState::Normal: return "NORMAL";
        case SafetyState::Warning: return "WARNING";
        case SafetyState::Surfacing: return "SURFACING";
        case SafetyState::FaultLatched: return "FAULT_LATCHED";
    }
    return "UNKNOWN";
}

SafetyMonitor::SafetyMonitor(SafetyLimits limits) : limits_(limits) {}

SafetyState SafetyMonitor::update(const SafetyInputs& in, double dt) {
    if (in.sensor_valid) {
        dropout_time_s_ = 0.0;
    } else {
        dropout_time_s_ += dt;
    }

    if (state_ == SafetyState::FaultLatched) return state_;  // terminal state

    if (state_ == SafetyState::Surfacing) {
        if (in.at_surface) state_ = SafetyState::FaultLatched;
        return state_;  // Surfacing never de-escalates
    }

    const bool surface_needed = in.depth_m > limits_.max_depth_m ||
                                in.battery_frac <= limits_.battery_surface ||
                                dropout_time_s_ >= limits_.dropout_surface_s;
    if (surface_needed) {
        state_ = SafetyState::Surfacing;
        // Already at the surface when the fault triggers: nothing left to do but latch.
        if (in.at_surface) state_ = SafetyState::FaultLatched;
        return state_;
    }

    const bool warn = in.depth_m >= limits_.warn_depth_fraction * limits_.max_depth_m ||
                      in.battery_frac <= limits_.battery_warn ||
                      dropout_time_s_ >= limits_.dropout_warn_s;
    state_ = warn ? SafetyState::Warning : SafetyState::Normal;
    return state_;
}

double SafetyMonitor::effective_setpoint(double requested_m) const {
    if (state_ == SafetyState::Surfacing || state_ == SafetyState::FaultLatched) return 0.0;
    return requested_m;
}

}  // namespace rovsim
