#include <gtest/gtest.h>

#include <cmath>

#include "rovsim/pid_controller.hpp"

using namespace rovsim;

namespace {
PidGains gains(double kp, double ki, double kd, double lim = 100.0) {
    PidGains g;
    g.kp = kp;
    g.ki = ki;
    g.kd = kd;
    g.out_min = -lim;
    g.out_max = lim;
    return g;
}
}  // namespace

TEST(PidController, ProportionalOnlyOutputIsKpTimesError) {
    PidController pid(gains(2.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.update(10.0, 4.0, 0.1), 12.0);
}

TEST(PidController, OutputIsClampedToLimits) {
    PidController pid(gains(100.0, 0.0, 0.0, 5.0));
    EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, 0.1), 5.0);
    EXPECT_DOUBLE_EQ(pid.update(-10.0, 0.0, 0.1), -5.0);
}

TEST(PidController, IntegralAccumulatesForConstantError) {
    PidController pid(gains(0.0, 2.0, 0.0));
    double out = 0.0;
    for (int i = 0; i < 10; ++i) out = pid.update(1.0, 0.0, 0.1);  // 1 s of error 1
    EXPECT_NEAR(out, 2.0, 1e-9);
}

TEST(PidController, IntegralIsIndependentOfTimeStep) {
    PidController fast(gains(0.0, 2.0, 0.0)), slow(gains(0.0, 2.0, 0.0));
    double a = 0, b = 0;
    for (int i = 0; i < 100; ++i) a = fast.update(1.0, 0.0, 0.01);
    for (int i = 0; i < 10; ++i) b = slow.update(1.0, 0.0, 0.1);
    EXPECT_NEAR(a, b, 1e-9);
}

TEST(PidController, AntiWindupKeepsIntegratorBoundedWhileSaturated) {
    // Huge error for 60 s: the proportional term alone saturates the output.
    PidController pid(gains(10.0, 5.0, 0.0, 20.0));
    for (int i = 0; i < 600; ++i) pid.update(100.0, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(pid.last_output(), 20.0);
    EXPECT_NEAR(pid.integral(), 0.0, 1e-9);  // frozen, not wound up
    // When the error flips, the output must leave saturation immediately (no unwinding delay).
    const double out = pid.update(0.0, 5.0, 0.1);  // error -5 -> P = -50 -> saturates low
    EXPECT_DOUBLE_EQ(out, -20.0);
}

TEST(PidController, WithoutSaturationIntegratorStillWorks) {
    PidController pid(gains(1.0, 1.0, 0.0, 100.0));
    for (int i = 0; i < 50; ++i) pid.update(1.0, 0.0, 0.1);
    EXPECT_GT(pid.integral(), 4.0);
}

TEST(PidController, SetpointStepDoesNotKickTheDerivative) {
    // Derivative acts on the measurement: a constant measurement means D = 0 regardless of
    // the setpoint, so a setpoint jump changes the output only through the P term.
    PidController pid(gains(1.0, 0.0, 50.0));
    pid.update(0.0, 0.0, 0.1);
    const double out = pid.update(10.0, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 10.0);
}

TEST(PidController, DerivativeOpposesMeasurementMotion) {
    PidController pid(gains(0.0, 0.0, 10.0));
    pid.update(0.0, 0.0, 0.1);
    EXPECT_LT(pid.update(0.0, 1.0, 0.1), 0.0);  // measurement rising -> brake (negative)
}

TEST(PidController, NoDerivativeSpikeOnFirstSample) {
    PidController pid(gains(0.0, 0.0, 100.0));
    EXPECT_DOUBLE_EQ(pid.update(0.0, 123.0, 0.1), 0.0);
}

TEST(PidController, NonPositiveDtHoldsPreviousOutput) {
    PidController pid(gains(1.0, 0.0, 0.0));
    const double first = pid.update(3.0, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(pid.update(99.0, 0.0, 0.0), first);
    EXPECT_DOUBLE_EQ(pid.update(99.0, 0.0, -1.0), first);
}

TEST(PidController, ResetClearsState) {
    PidController pid(gains(1.0, 1.0, 1.0));
    for (int i = 0; i < 20; ++i) pid.update(5.0, 0.0, 0.1);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.last_output(), 0.0);
}

TEST(PidController, StepResponseOnFirstOrderPlantConverges) {
    // Plant: x' = u. PI control should drive x to the setpoint with no steady-state error.
    PidController pid(gains(2.0, 1.0, 0.0, 50.0));
    double x = 0.0;
    for (int i = 0; i < 2000; ++i) x += pid.update(3.0, x, 0.01) * 0.01;
    EXPECT_NEAR(x, 3.0, 1e-3);
}
