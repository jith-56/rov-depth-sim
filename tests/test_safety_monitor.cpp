#include <gtest/gtest.h>

#include "rovsim/safety_monitor.hpp"

using namespace rovsim;

namespace {
SafetyInputs ok(double depth = 5.0, double battery = 0.9) {
    SafetyInputs in;
    in.depth_m = depth;
    in.battery_frac = battery;
    in.sensor_valid = true;
    in.at_surface = false;
    return in;
}
SafetyInputs no_sensor(double depth = 5.0) {
    SafetyInputs in = ok(depth);
    in.sensor_valid = false;
    return in;
}
// Feed `seconds` of the same input at 0.05 s steps.
SafetyState feed(SafetyMonitor& m, const SafetyInputs& in, double seconds) {
    SafetyState s = m.state();
    for (int i = 0; i < static_cast<int>(seconds / 0.05 + 0.5); ++i) s = m.update(in, 0.05);
    return s;
}
}  // namespace

// ---- NORMAL ----
TEST(SafetyMonitor, StartsNormalAndStaysNormalWhenHealthy) {
    SafetyMonitor m;
    EXPECT_EQ(m.state(), SafetyState::Normal);
    EXPECT_EQ(feed(m, ok(), 30.0), SafetyState::Normal);
}

// ---- NORMAL -> WARNING ----
TEST(SafetyMonitor, DepthAtWarnFractionRaisesWarning) {
    SafetyMonitor m;  // max 10 m, warn at 9 m
    EXPECT_EQ(feed(m, ok(8.9), 1.0), SafetyState::Normal);
    EXPECT_EQ(feed(m, ok(9.0), 0.1), SafetyState::Warning);
}

TEST(SafetyMonitor, LowBatteryRaisesWarningAtTwentyPercent) {
    SafetyMonitor m;
    EXPECT_EQ(feed(m, ok(5.0, 0.21), 1.0), SafetyState::Normal);
    EXPECT_EQ(feed(m, ok(5.0, 0.20), 0.1), SafetyState::Warning);
}

TEST(SafetyMonitor, SensorSilenceRaisesWarningAfterTwoSeconds) {
    SafetyMonitor m;
    EXPECT_EQ(feed(m, no_sensor(), 1.9), SafetyState::Normal);
    EXPECT_EQ(feed(m, no_sensor(), 0.2), SafetyState::Warning);
}

// ---- WARNING -> NORMAL (soft limits clear) ----
TEST(SafetyMonitor, WarningClearsWhenSensorReturns) {
    SafetyMonitor m;
    feed(m, no_sensor(), 3.0);
    ASSERT_EQ(m.state(), SafetyState::Warning);
    EXPECT_EQ(feed(m, ok(), 0.1), SafetyState::Normal);
    EXPECT_DOUBLE_EQ(m.dropout_time_s(), 0.0);
}

TEST(SafetyMonitor, DepthWarningClearsWhenShallower) {
    SafetyMonitor m;
    feed(m, ok(9.5), 0.5);
    ASSERT_EQ(m.state(), SafetyState::Warning);
    EXPECT_EQ(feed(m, ok(6.0), 0.1), SafetyState::Normal);
}

// ---- -> SURFACING (hard limits) ----
TEST(SafetyMonitor, DepthBeyondMaxForcesSurfacing) {
    SafetyMonitor m;
    EXPECT_EQ(feed(m, ok(10.0), 0.5), SafetyState::Warning);  // exactly at the limit: not over
    EXPECT_EQ(feed(m, ok(10.01), 0.1), SafetyState::Surfacing);
}

TEST(SafetyMonitor, BatteryAtTenPercentForcesSurfacing) {
    SafetyMonitor m;
    EXPECT_EQ(feed(m, ok(5.0, 0.11), 0.5), SafetyState::Warning);
    EXPECT_EQ(feed(m, ok(5.0, 0.10), 0.1), SafetyState::Surfacing);
}

TEST(SafetyMonitor, SensorSilenceForFiveSecondsForcesSurfacing) {
    SafetyMonitor m;
    EXPECT_EQ(feed(m, no_sensor(), 4.9), SafetyState::Warning);
    EXPECT_EQ(feed(m, no_sensor(), 0.2), SafetyState::Surfacing);
}

TEST(SafetyMonitor, WarningCanEscalateDirectlyAcrossLevels) {
    SafetyMonitor m;
    EXPECT_EQ(feed(m, ok(5.0, 0.05), 0.1), SafetyState::Surfacing);  // skips straight past WARNING
}

// ---- SURFACING is latched ----
TEST(SafetyMonitor, SurfacingNeverDeEscalatesEvenWhenConditionClears) {
    SafetyMonitor m;
    feed(m, no_sensor(), 6.0);
    ASSERT_EQ(m.state(), SafetyState::Surfacing);
    EXPECT_EQ(feed(m, ok(3.0, 0.9), 5.0), SafetyState::Surfacing);  // sensor is back, still latched
}

// ---- SURFACING -> FAULT_LATCHED ----
TEST(SafetyMonitor, ReachingSurfaceLatchesTheFault) {
    SafetyMonitor m;
    feed(m, ok(11.0), 0.1);
    ASSERT_EQ(m.state(), SafetyState::Surfacing);
    SafetyInputs at_top = ok(0.1);
    at_top.at_surface = true;
    EXPECT_EQ(feed(m, at_top, 0.1), SafetyState::FaultLatched);
}

TEST(SafetyMonitor, FaultTriggeredAtTheSurfaceLatchesImmediately) {
    SafetyMonitor m;
    SafetyInputs in = ok(0.0, 0.05);
    in.at_surface = true;
    EXPECT_EQ(feed(m, in, 0.1), SafetyState::FaultLatched);
}

TEST(SafetyMonitor, FaultLatchedIsTerminal) {
    SafetyMonitor m;
    SafetyInputs in = ok(0.0, 0.05);
    in.at_surface = true;
    feed(m, in, 0.1);
    ASSERT_EQ(m.state(), SafetyState::FaultLatched);
    EXPECT_EQ(feed(m, ok(5.0, 1.0), 10.0), SafetyState::FaultLatched);
}

// ---- Outputs ----
TEST(SafetyMonitor, EffectiveSetpointFollowsOperatorUntilSurfacing) {
    SafetyMonitor m;
    EXPECT_DOUBLE_EQ(m.effective_setpoint(7.0), 7.0);
    feed(m, ok(9.5), 0.5);  // WARNING: operator still in control
    EXPECT_DOUBLE_EQ(m.effective_setpoint(7.0), 7.0);
    feed(m, ok(11.0), 0.1);  // SURFACING: forced to 0 m
    EXPECT_DOUBLE_EQ(m.effective_setpoint(7.0), 0.0);
}

TEST(SafetyMonitor, ThrustDisabledOnlyWhenFaultLatched) {
    SafetyMonitor m;
    EXPECT_TRUE(m.thrust_enabled());
    feed(m, ok(11.0), 0.1);
    EXPECT_TRUE(m.thrust_enabled());  // surfacing needs thrust
    SafetyInputs at_top = ok(0.1);
    at_top.at_surface = true;
    feed(m, at_top, 0.1);
    EXPECT_FALSE(m.thrust_enabled());
}

TEST(SafetyMonitor, StateNamesAreStable) {
    EXPECT_STREQ(to_string(SafetyState::Normal), "NORMAL");
    EXPECT_STREQ(to_string(SafetyState::Warning), "WARNING");
    EXPECT_STREQ(to_string(SafetyState::Surfacing), "SURFACING");
    EXPECT_STREQ(to_string(SafetyState::FaultLatched), "FAULT_LATCHED");
}
