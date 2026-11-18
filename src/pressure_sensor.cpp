#include "rovsim/pressure_sensor.hpp"

#include <algorithm>

namespace rovsim {

PressureSensor::PressureSensor(SensorParams params)
    : params_(params), rng_(params.seed), noise_(0.0, std::max(params.noise_std_m, 0.0)) {}

std::optional<double> PressureSensor::read(double true_depth_m) {
    if (dropout_reads_left_ > 0) {
        --dropout_reads_left_;
        return std::nullopt;
    }
    const double noise = params_.noise_std_m > 0.0 ? noise_(rng_) : 0.0;
    return true_depth_m + params_.bias_m + noise;
}

void PressureSensor::start_dropout(int n_reads) {
    dropout_reads_left_ = std::max(n_reads, 0);
}

}  // namespace rovsim
