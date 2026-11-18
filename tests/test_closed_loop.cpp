// Integration tests: the full loop (model + noisy sensor + PID + safety + battery).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>

#include "rovsim/simulation.hpp"

using namespace rovsim;

namespace {
Scenario scenario(const std::string& name) {
    Scenario s;
    EXPECT_TRUE(make_scenario(name, s)) << name;
    return s;
}
double max_depth(const std::vector<LogRow>& rows) {
    double m = 0.0;
    for (const auto& r : rows) m = std::max(m, r.true_depth_m);
    return m;
}
bool ever_in_state(const std::vector<LogRow>& rows, SafetyState s) {
    return std::any_of(rows.begin(), rows.end(), [&](const LogRow& r) { return r.state == s; });
}
// Last time the true depth was outside +-band of the target.
double settling_time(const std::vector<LogRow>& rows, double target, double band) {
    double t = 0.0;
    for (const auto& r : rows)
        if (std::fabs(r.true_depth_m - target) > band) t = r.time_s;
    return t;
}
}  // namespace

TEST(ClosedLoop, NominalHoldsFiveMetresWithSmallOvershoot) {
    const auto rows = run_scenario(scenario("nominal"));
    EXPECT_LT(max_depth(rows), 5.0 * 1.05);                    // overshoot < 5%
    EXPECT_LT(settling_time(rows, 5.0, 0.1), 10.0);            // within 2% by 10 s
    EXPECT_NEAR(rows.back().true_depth_m, 5.0, 0.05);          // steady-state error
    EXPECT_FALSE(ever_in_state(rows, SafetyState::Warning));   // healthy run raises no alarms
    EXPECT_EQ(rows.back().state, SafetyState::Normal);
}

TEST(ClosedLoop, NominalThrustStaysWithinLimitsAndIsNotJittery) {
    const auto rows = run_scenario(scenario("nominal"));
    double lo = 1e9, hi = -1e9;
    for (const auto& r : rows) {
        lo = std::min(lo, r.thrust_n);
        hi = std::max(hi, r.thrust_n);
    }
    EXPECT_GE(lo, -40.0);
    EXPECT_LE(hi, 40.0);
    // In the last 10 s the command is steady (noise rejection through the filtered D term).
    double mn = 1e9, mx = -1e9;
    for (std::size_t i = rows.size() - 100; i < rows.size(); ++i) {
        mn = std::min(mn, rows[i].thrust_n);
        mx = std::max(mx, rows[i].thrust_n);
    }
    EXPECT_LT(mx - mn, 10.0);
}

TEST(ClosedLoop, RejectsDifferentCurrentDisturbancesAndSensorBias) {
    for (double current : {-3.0, 0.0, 1.5, 4.0}) {
        Scenario s = scenario("nominal");
        s.current_force_n = current;
        s.duration_s = 80.0;
        const auto rows = run_scenario(s);
        EXPECT_NEAR(rows.back().true_depth_m, 5.0, 0.1) << "current " << current;
        EXPECT_LT(max_depth(rows), 5.0 * 1.10) << "current " << current;
    }
}

TEST(ClosedLoop, ShortDropoutWarnsThenRecoversAndKeepsDepth) {
    const auto rows = run_scenario(scenario("sensor_dropout"));
    EXPECT_TRUE(ever_in_state(rows, SafetyState::Warning));
    EXPECT_FALSE(ever_in_state(rows, SafetyState::Surfacing));
    EXPECT_EQ(rows.back().state, SafetyState::Normal);
    EXPECT_NEAR(rows.back().true_depth_m, 5.0, 0.1);
}

TEST(ClosedLoop, DropoutRowsHaveNoMeasurement) {
    const auto rows = run_scenario(scenario("sensor_dropout"));
    const auto missing = std::count_if(rows.begin(), rows.end(),
                                       [](const LogRow& r) { return std::isnan(r.measured_depth_m); });
    EXPECT_GT(missing, 20);  // ~3 s at 10 Hz logging
    EXPECT_LT(missing, 40);
}

TEST(ClosedLoop, LongSensorLossSurfacesBlindAndLatches) {
    const auto rows = run_scenario(scenario("sensor_loss"));
    EXPECT_TRUE(ever_in_state(rows, SafetyState::Warning));
    EXPECT_TRUE(ever_in_state(rows, SafetyState::Surfacing));
    EXPECT_EQ(rows.back().state, SafetyState::FaultLatched);
    EXPECT_NEAR(rows.back().true_depth_m, 0.0, 0.01);
    EXPECT_NEAR(rows.back().thrust_n, 0.0, 1e-9);  // thrusters off once latched
}

TEST(ClosedLoop, LowBatteryWarnsThenSurfacesThenLatches) {
    const auto rows = run_scenario(scenario("low_battery"));
    SafetyState prev = rows.front().state;
    std::vector<SafetyState> order{prev};
    for (const auto& r : rows)
        if (r.state != prev) order.push_back(prev = r.state);
    ASSERT_EQ(order.size(), 4u);
    EXPECT_EQ(order[0], SafetyState::Normal);
    EXPECT_EQ(order[1], SafetyState::Warning);
    EXPECT_EQ(order[2], SafetyState::Surfacing);
    EXPECT_EQ(order[3], SafetyState::FaultLatched);
    EXPECT_NEAR(rows.back().true_depth_m, 0.0, 0.01);
}

TEST(ClosedLoop, OperatorRequestBeyondDepthLimitIsOverriddenAndVehicleSurfaces) {
    const auto rows = run_scenario(scenario("depth_limit"));  // operator asks for 12 m, limit 10 m
    EXPECT_TRUE(ever_in_state(rows, SafetyState::Warning));
    EXPECT_TRUE(ever_in_state(rows, SafetyState::Surfacing));
    EXPECT_LT(max_depth(rows), 10.5);  // never goes meaningfully past the hard limit
    EXPECT_EQ(rows.back().state, SafetyState::FaultLatched);
    // The effective setpoint was forced to 0 m while the requested one stayed 12 m.
    EXPECT_DOUBLE_EQ(rows.back().effective_setpoint_m, 0.0);
    EXPECT_DOUBLE_EQ(rows.back().requested_setpoint_m, 12.0);
}

TEST(ClosedLoop, SimulationIsDeterministic) {
    const auto a = run_scenario(scenario("nominal"));
    const auto b = run_scenario(scenario("nominal"));
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) EXPECT_DOUBLE_EQ(a[i].true_depth_m, b[i].true_depth_m);
}

TEST(ClosedLoop, LogHasExpectedLengthAndTimeBase) {
    const auto rows = run_scenario(scenario("nominal"));  // 60 s at 10 Hz + the t = 0 row
    EXPECT_EQ(rows.size(), 601u);
    EXPECT_DOUBLE_EQ(rows.front().time_s, 0.0);
    EXPECT_NEAR(rows.back().time_s, 60.0, 1e-9);
}

TEST(ClosedLoop, UnknownScenarioNameIsRejected) {
    Scenario s;
    EXPECT_FALSE(make_scenario("does_not_exist", s));
}

TEST(ClosedLoop, CsvWriterProducesHeaderAndAllRows) {
    const auto rows = run_scenario(scenario("sensor_dropout"));
    const std::string path = "rovsim_test_output.csv";
    ASSERT_TRUE(write_csv(path, rows));
    std::ifstream f(path);
    std::string line;
    std::getline(f, line);
    EXPECT_EQ(line.rfind("time_s,true_depth_m,measured_depth_m", 0), 0u);
    std::size_t n = 0;
    while (std::getline(f, line)) ++n;
    EXPECT_EQ(n, rows.size());
    f.close();
    std::remove(path.c_str());
}
