#pragma once

#include <string>
#include <vector>

#include "rovsim/pid_controller.hpp"
#include "rovsim/pressure_sensor.hpp"
#include "rovsim/rov_model.hpp"
#include "rovsim/safety_monitor.hpp"

namespace rovsim {

// Everything that defines one simulated mission.
struct Scenario {
    std::string name = "nominal";
    double duration_s = 60.0;
    double target_depth_m = 5.0;        // operator's requested depth
    double current_force_n = 1.5;       // vertical disturbance (downwelling)
    SensorParams sensor{};
    double dropout_start_s = -1.0;      // < 0 means no dropout
    double dropout_duration_s = 0.0;
    double battery_capacity_j = 36000.0;
    double battery_initial = 1.0;
    double ascent_thrust_n = -20.0;     // open-loop thrust used to surface with no sensor
    SafetyLimits safety{};
    PidGains gains{};
};

// One line of the log (written every 0.1 s of simulated time).
struct LogRow {
    double time_s;
    double true_depth_m;
    double measured_depth_m;     // NaN while the sensor is in dropout
    double requested_setpoint_m;
    double effective_setpoint_m;
    double thrust_n;
    double battery_frac;
    SafetyState state;
};

// Default controller tuning used by every scenario.
PidGains default_gains();

std::vector<std::string> scenario_names();
// Returns false (and leaves `out` untouched) if the name is unknown.
bool make_scenario(const std::string& name, Scenario& out);

// Runs the closed loop: plant at 100 Hz, sensor + controller + safety at 20 Hz.
std::vector<LogRow> run_scenario(const Scenario& scenario);

bool write_csv(const std::string& path, const std::vector<LogRow>& rows);

}  // namespace rovsim
