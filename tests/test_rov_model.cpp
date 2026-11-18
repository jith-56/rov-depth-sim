#include <gtest/gtest.h>

#include <cmath>

#include "rovsim/rov_model.hpp"

using namespace rovsim;

namespace {
void run(RovModel& m, double thrust, double seconds, double dt = 0.01) {
    const long n = std::lround(seconds / dt);
    for (long i = 0; i < n; ++i) m.step(thrust, dt);
}
}  // namespace

TEST(RovModel, ZeroThrustRisesAtTerminalVelocity) {
    // Net buoyancy 2 N up, drag 20 N/(m/s)^2 -> terminal speed = sqrt(2/20) = 0.316 m/s up.
    RovModel m(RovParams{}, RovState{50.0, 0.0});
    run(m, 0.0, 15.0);
    EXPECT_NEAR(m.state().velocity_mps, -std::sqrt(2.0 / 20.0), 0.005);
    EXPECT_LT(m.state().depth_m, 50.0);
}

TEST(RovModel, CurrentDisturbanceChangesEquilibrium) {
    // A 5 N downward current beats the 2 N buoyancy: net 3 N down -> sinks at sqrt(3/20).
    RovParams p;
    p.current_force_n = 5.0;
    RovModel m(p);
    run(m, 0.0, 15.0);
    EXPECT_NEAR(m.state().velocity_mps, std::sqrt(3.0 / 20.0), 0.005);
}

TEST(RovModel, ThrustIsClampedToLimit) {
    RovModel m;
    m.step(1000.0, 0.01);
    EXPECT_DOUBLE_EQ(m.applied_thrust(), m.params().max_thrust_n);
    m.step(-1000.0, 0.01);
    EXPECT_DOUBLE_EQ(m.applied_thrust(), -m.params().max_thrust_n);
}

TEST(RovModel, DownThrustMovesDown) {
    RovModel m;
    run(m, 20.0, 5.0);
    EXPECT_GT(m.state().depth_m, 1.0);
    EXPECT_GT(m.state().velocity_mps, 0.0);
}

TEST(RovModel, DragOpposesMotionAndBoundsSpeed) {
    // Full thrust down: terminal speed = sqrt((40 - 2) / 20) = 1.378 m/s. Never exceeded.
    RovModel m;
    double vmax = 0.0;
    for (int i = 0; i < 3000; ++i) {
        m.step(40.0, 0.01);
        vmax = std::fmax(vmax, m.state().velocity_mps);
    }
    EXPECT_NEAR(vmax, std::sqrt(38.0 / 20.0), 0.01);
}

TEST(RovModel, SurfaceIsAHardWall) {
    RovModel m;  // starts at the surface
    run(m, -40.0, 5.0);
    EXPECT_DOUBLE_EQ(m.state().depth_m, 0.0);
    EXPECT_GE(m.state().velocity_mps, 0.0);
}

TEST(RovModel, BuoyantVehicleFloatsToSurfaceWithThrustersOff) {
    RovModel m(RovParams{}, RovState{3.0, 0.0});
    run(m, 0.0, 30.0);
    EXPECT_DOUBLE_EQ(m.state().depth_m, 0.0);
}
