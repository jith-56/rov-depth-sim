#include "rovsim/rov_model.hpp"

#include <algorithm>
#include <cmath>

namespace rovsim {

RovModel::RovModel(RovParams params, RovState initial) : params_(params), state_(initial) {}

void RovModel::step(double thrust_cmd_n, double dt) {
    applied_thrust_ = std::clamp(thrust_cmd_n, -params_.max_thrust_n, params_.max_thrust_n);

    const double v = state_.velocity_mps;
    const double drag = -params_.drag_coeff * v * std::fabs(v);  // always opposes motion
    const double force = applied_thrust_ + params_.net_weight_n + params_.current_force_n + drag;
    const double accel = force / params_.mass_kg;

    state_.velocity_mps += accel * dt;
    state_.depth_m += state_.velocity_mps * dt;

    // The surface is a wall: the vehicle cannot rise above depth 0.
    if (state_.depth_m < 0.0) {
        state_.depth_m = 0.0;
        if (state_.velocity_mps < 0.0) state_.velocity_mps = 0.0;
    }
}

}  // namespace rovsim
