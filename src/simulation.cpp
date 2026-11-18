#include "rovsim/simulation.hpp"

#include <cmath>
#include <fstream>
#include <limits>

#include "rovsim/battery_model.hpp"

namespace rovsim {

namespace {
constexpr double kPlantDt = 0.01;     // physics step: 100 Hz
constexpr int kControlEvery = 5;      // sensor + controller + safety: 20 Hz
constexpr int kLogEvery = 10;         // log: 10 Hz
constexpr double kControlDt = kPlantDt * kControlEvery;
constexpr double kSurfaceContactM = 0.2;  // true depth below which the surface flag is set
}  // namespace

PidGains default_gains() {
    PidGains g;
    // Chosen by a grid search over (kp, ki, kd, filter tau) on the nominal 0 -> 5 m step,
    // minimising thrust jitter subject to overshoot < 5% and 2% settling time < 10 s.
    g.kp = 30.0;
    g.ki = 1.0;
    g.kd = 15.0;
    g.out_min = -40.0;
    g.out_max = 40.0;
    g.d_filter_tau_s = 0.5;
    return g;
}

std::vector<std::string> scenario_names() {
    return {"nominal", "sensor_dropout", "sensor_loss", "low_battery", "depth_limit"};
}

bool make_scenario(const std::string& name, Scenario& out) {
    Scenario s;
    s.name = name;
    s.gains = default_gains();
    if (name == "nominal") {
        s.duration_s = 60.0;
        s.target_depth_m = 5.0;
    } else if (name == "sensor_dropout") {
        // A 3 s sensor outage: long enough for WARNING (>= 2 s), too short for SURFACING.
        s.duration_s = 60.0;
        s.target_depth_m = 5.0;
        s.dropout_start_s = 30.0;
        s.dropout_duration_s = 3.0;
    } else if (name == "sensor_loss") {
        // A 7 s outage: crosses the 5 s hard limit, so the vehicle must surface open-loop.
        s.duration_s = 90.0;
        s.target_depth_m = 5.0;
        s.dropout_start_s = 30.0;
        s.dropout_duration_s = 7.0;
    } else if (name == "low_battery") {
        // Small battery starting at 24%: crosses 20% (WARNING) then 10% (SURFACING).
        s.duration_s = 120.0;
        s.target_depth_m = 5.0;
        s.battery_capacity_j = 10000.0;
        s.battery_initial = 0.24;
    } else if (name == "depth_limit") {
        // Operator error: requests 12 m, beyond the 10 m limit. Must warn, then surface.
        s.duration_s = 90.0;
        s.target_depth_m = 12.0;
    } else {
        return false;
    }
    out = s;
    return true;
}

std::vector<LogRow> run_scenario(const Scenario& sc) {
    RovParams rov_params;
    rov_params.current_force_n = sc.current_force_n;
    RovModel rov(rov_params);
    PressureSensor sensor(sc.sensor);
    PidController pid(sc.gains);
    SafetyMonitor safety(sc.safety);
    BatteryModel battery(sc.battery_capacity_j, sc.battery_initial);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const long total_steps = std::lround(sc.duration_s / kPlantDt);

    std::vector<LogRow> rows;
    rows.reserve(static_cast<std::size_t>(total_steps / kLogEvery) + 1);

    double last_measurement = 0.0;   // last valid depth reading (held during dropouts)
    double measured_for_log = 0.0;   // NaN when the latest cycle had no reading
    double thrust_cmd = 0.0;
    double effective_sp = sc.target_depth_m;
    bool dropout_started = false;

    for (long k = 0; k <= total_steps; ++k) {
        const double t = static_cast<double>(k) * kPlantDt;

        if (k % kControlEvery == 0) {
            // 1. Schedule the sensor dropout when its start time is reached.
            if (sc.dropout_start_s >= 0.0 && !dropout_started && t >= sc.dropout_start_s) {
                sensor.start_dropout(static_cast<int>(std::lround(sc.dropout_duration_s / kControlDt)));
                dropout_started = true;
            }

            // 2. Read the sensor.
            const auto reading = sensor.read(rov.state().depth_m);
            const bool valid = reading.has_value();
            if (valid) last_measurement = *reading;
            measured_for_log = valid ? *reading : nan;

            // 3. Safety supervisor.
            SafetyInputs in;
            in.depth_m = last_measurement;
            in.battery_frac = battery.fraction();
            in.sensor_valid = valid;
            in.at_surface = rov.state().depth_m <= kSurfaceContactM;
            safety.update(in, kControlDt);
            effective_sp = safety.effective_setpoint(sc.target_depth_m);

            // 4. Controller output.
            if (!safety.thrust_enabled()) {
                pid.reset();
                thrust_cmd = 0.0;  // fault latched: thrusters off, positive buoyancy floats us
            } else if (valid) {
                thrust_cmd = pid.update(effective_sp, last_measurement, kControlDt);
            } else if (safety.state() == SafetyState::Surfacing) {
                thrust_cmd = sc.ascent_thrust_n;  // blind: open-loop ascent
            }
            // else: short dropout, hold the previous thrust command
        }

        if (k % kLogEvery == 0) {
            rows.push_back({t, rov.state().depth_m, measured_for_log, sc.target_depth_m,
                            effective_sp, rov.applied_thrust(), battery.fraction(),
                            safety.state()});
        }

        if (k < total_steps) {
            rov.step(thrust_cmd, kPlantDt);
            battery.step(rov.applied_thrust(), kPlantDt);
        }
    }
    return rows;
}

bool write_csv(const std::string& path, const std::vector<LogRow>& rows) {
    std::ofstream f(path);
    if (!f) return false;
    f << "time_s,true_depth_m,measured_depth_m,requested_setpoint_m,effective_setpoint_m,"
         "thrust_n,battery_frac,state\n";
    f.setf(std::ios::fixed);
    f.precision(4);
    for (const auto& r : rows) {
        f << r.time_s << ',' << r.true_depth_m << ',';
        if (!std::isnan(r.measured_depth_m)) f << r.measured_depth_m;
        f << ',' << r.requested_setpoint_m << ',' << r.effective_setpoint_m << ',' << r.thrust_n
          << ',' << r.battery_frac << ',' << to_string(r.state) << '\n';
    }
    return static_cast<bool>(f);
}

}  // namespace rovsim
