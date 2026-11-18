#pragma once

#include <optional>
#include <random>

namespace rovsim {

struct SensorParams {
    double noise_std_m = 0.02;  // 1-sigma Gaussian noise on the depth reading
    double bias_m = 0.0;        // constant calibration offset
    unsigned seed = 42;         // fixed seed -> deterministic tests and reproducible logs
};

// Models a pressure-based depth sensor: measured = true + bias + noise.
// A dropout makes read() return std::nullopt for a number of consecutive reads.
class PressureSensor {
public:
    explicit PressureSensor(SensorParams params = {});

    std::optional<double> read(double true_depth_m);

    // The next `n_reads` calls to read() return no value.
    void start_dropout(int n_reads);
    bool in_dropout() const { return dropout_reads_left_ > 0; }

private:
    SensorParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_;
    int dropout_reads_left_ = 0;
};

}  // namespace rovsim
