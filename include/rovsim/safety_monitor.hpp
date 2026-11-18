#pragma once

namespace rovsim {

enum class SafetyState {
    Normal = 0,       // all checks pass
    Warning = 1,      // a soft limit is exceeded; mission continues, operator is alerted
    Surfacing = 2,    // a hard limit is exceeded; setpoint forced to 0 m. Latched: no way back
    FaultLatched = 3  // vehicle reached the surface after a fault; thrusters off, mission over
};

const char* to_string(SafetyState s);

struct SafetyLimits {
    double max_depth_m = 10.0;           // hard depth limit -> Surfacing
    double warn_depth_fraction = 0.9;    // soft depth limit (fraction of max) -> Warning
    double battery_warn = 0.20;          // state of charge <= this -> Warning
    double battery_surface = 0.10;       // state of charge <= this -> Surfacing
    double dropout_warn_s = 2.0;         // sensor silent for >= this long -> Warning
    double dropout_surface_s = 5.0;      // sensor silent for >= this long -> Surfacing
};

struct SafetyInputs {
    double depth_m = 0.0;        // best depth estimate (last valid sensor reading)
    double battery_frac = 1.0;   // state of charge in [0, 1]
    bool sensor_valid = true;    // did the depth sensor deliver a reading this cycle?
    bool at_surface = false;     // independent surface-contact flag (not the depth sensor)
};

// Fail-safe supervisor. Escalation is one-way for hard faults: once Surfacing, the monitor
// never returns to Normal/Warning by itself (a vehicle that lost its sensor and came back
// should not resume the mission automatically). Warning, however, clears when the
// condition clears.
class SafetyMonitor {
public:
    explicit SafetyMonitor(SafetyLimits limits = {});

    SafetyState update(const SafetyInputs& in, double dt);

    SafetyState state() const { return state_; }
    double dropout_time_s() const { return dropout_time_s_; }

    // The setpoint the controller must track: the operator's request, unless a fault
    // forces the vehicle to the surface.
    double effective_setpoint(double requested_m) const;

    // False once the fault is latched: thrusters are disabled.
    bool thrust_enabled() const { return state_ != SafetyState::FaultLatched; }

private:
    SafetyLimits limits_;
    SafetyState state_ = SafetyState::Normal;
    double dropout_time_s_ = 0.0;
};

}  // namespace rovsim
