#include "rovsim/battery_model.hpp"

#include <algorithm>

namespace rovsim {

BatteryModel::BatteryModel(double capacity_j, double initial_fraction, double idle_w,
                           double thrust_w_per_n2)
    : capacity_j_(std::max(capacity_j, 1.0)),
      used_j_(capacity_j_ * (1.0 - std::clamp(initial_fraction, 0.0, 1.0))),
      idle_w_(idle_w),
      thrust_w_per_n2_(thrust_w_per_n2) {}

double BatteryModel::power_w(double thrust_n) const {
    return idle_w_ + thrust_w_per_n2_ * thrust_n * thrust_n;
}

void BatteryModel::step(double thrust_n, double dt) {
    used_j_ += power_w(thrust_n) * dt;
}

double BatteryModel::fraction() const {
    return std::clamp(1.0 - used_j_ / capacity_j_, 0.0, 1.0);
}

}  // namespace rovsim
