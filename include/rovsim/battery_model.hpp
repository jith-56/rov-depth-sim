#pragma once

namespace rovsim {

// Simple energy model: power = idle load + k * thrust^2 (thruster power grows with effort).
// State of charge is the fraction of capacity remaining, clamped to [0, 1].
class BatteryModel {
public:
    explicit BatteryModel(double capacity_j = 36000.0, double initial_fraction = 1.0,
                          double idle_w = 20.0, double thrust_w_per_n2 = 0.02);

    void step(double thrust_n, double dt);

    double fraction() const;
    double power_w(double thrust_n) const;

private:
    double capacity_j_;
    double used_j_;
    double idle_w_;
    double thrust_w_per_n2_;
};

}  // namespace rovsim
