#pragma once

namespace rovsim {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double out_min = -1.0;
    double out_max = 1.0;
    double d_filter_tau_s = 0.25;  // time constant of the low-pass on the derivative term
};

// PID controller with the three details that matter on real hardware:
//  * Anti-windup: the integrator is frozen while the output is saturated and the error
//    would drive it further into saturation (conditional integration).
//  * Derivative on measurement (not on error): a setpoint step does not cause a "kick".
//  * Filtered derivative: a first-order low-pass keeps sensor noise out of the output.
// The time step is an argument of update(), so it can change between calls.
class PidController {
public:
    explicit PidController(PidGains gains = {});

    double update(double setpoint, double measurement, double dt);
    void reset();

    double integral() const { return integral_; }
    double last_output() const { return last_output_; }
    const PidGains& gains() const { return gains_; }

private:
    PidGains gains_;
    double integral_ = 0.0;      // already multiplied by ki, so it is in output units
    double prev_measurement_ = 0.0;
    double d_filtered_ = 0.0;    // filtered d(measurement)/dt
    double last_output_ = 0.0;
    bool first_call_ = true;
};

}  // namespace rovsim
