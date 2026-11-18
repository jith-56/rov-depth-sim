#pragma once

namespace rovsim {

// Physical parameters of the simulated vehicle (1-D vertical motion).
// Sign convention: depth and velocity are positive DOWN, thrust is positive DOWN.
struct RovParams {
    double mass_kg = 15.0;          // vehicle mass including added (hydrodynamic) mass
    double net_weight_n = -2.0;     // weight - buoyancy; negative = positively buoyant (floats up)
    double drag_coeff = 20.0;       // quadratic drag coefficient [N / (m/s)^2]
    double max_thrust_n = 40.0;     // thruster saturation limit (symmetric)
    double current_force_n = 0.0;   // constant vertical disturbance force (+ = pushes down)
};

struct RovState {
    double depth_m = 0.0;
    double velocity_mps = 0.0;
};

// Newton's second law for a vehicle in water:
//   m * dv/dt = thrust + net_weight + current - c_d * v * |v|
// integrated with semi-implicit Euler (velocity first, then position), which is stable
// for the stiff drag term at our fixed step. The water surface (depth 0) is a hard wall.
class RovModel {
public:
    explicit RovModel(RovParams params = {}, RovState initial = {});

    void step(double thrust_cmd_n, double dt);

    const RovState& state() const { return state_; }
    const RovParams& params() const { return params_; }
    double applied_thrust() const { return applied_thrust_; }
    void set_current_force(double force_n) { params_.current_force_n = force_n; }

private:
    RovParams params_;
    RovState state_;
    double applied_thrust_ = 0.0;
};

}  // namespace rovsim
