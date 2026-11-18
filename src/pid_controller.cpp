#include "rovsim/pid_controller.hpp"

#include <algorithm>

namespace rovsim {

PidController::PidController(PidGains gains) : gains_(gains) {}

void PidController::reset() {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    d_filtered_ = 0.0;
    last_output_ = 0.0;
    first_call_ = true;
}

double PidController::update(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return last_output_;  // invalid step: hold the previous command

    const double error = setpoint - measurement;

    // Derivative on measurement: d(error)/dt = -d(measurement)/dt for a constant setpoint.
    if (first_call_) {
        prev_measurement_ = measurement;  // avoid a derivative spike on the first sample
        first_call_ = false;
    }
    const double d_raw = (measurement - prev_measurement_) / dt;
    prev_measurement_ = measurement;
    const double alpha = dt / (gains_.d_filter_tau_s + dt);  // first-order low-pass
    d_filtered_ += alpha * (d_raw - d_filtered_);

    const double p_term = gains_.kp * error;
    const double d_term = -gains_.kd * d_filtered_;

    // Conditional integration (anti-windup).
    const double candidate = integral_ + gains_.ki * error * dt;
    const double unsaturated = p_term + candidate + d_term;
    const bool pushing_high = unsaturated > gains_.out_max && error > 0.0;
    const bool pushing_low = unsaturated < gains_.out_min && error < 0.0;
    if (!pushing_high && !pushing_low) {
        integral_ = std::clamp(candidate, gains_.out_min, gains_.out_max);
    }

    last_output_ = std::clamp(p_term + integral_ + d_term, gains_.out_min, gains_.out_max);
    return last_output_;
}

}  // namespace rovsim
