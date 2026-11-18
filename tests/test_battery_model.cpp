#include <gtest/gtest.h>

#include "rovsim/battery_model.hpp"

using namespace rovsim;

TEST(BatteryModel, StartsAtRequestedFraction) {
    BatteryModel b(1000.0, 0.6);
    EXPECT_NEAR(b.fraction(), 0.6, 1e-12);
}

TEST(BatteryModel, DrainsAtIdlePowerWithNoThrust) {
    BatteryModel b(1000.0, 1.0, 20.0, 0.02);
    for (int i = 0; i < 100; ++i) b.step(0.0, 0.1);  // 10 s at 20 W = 200 J
    EXPECT_NEAR(b.fraction(), 0.8, 1e-9);
}

TEST(BatteryModel, ThrusterEffortDrainsFaster) {
    BatteryModel idle(1000.0), busy(1000.0);
    for (int i = 0; i < 100; ++i) {
        idle.step(0.0, 0.1);
        busy.step(40.0, 0.1);
    }
    EXPECT_LT(busy.fraction(), idle.fraction());
}

TEST(BatteryModel, PowerGrowsWithThrustMagnitudeNotSign) {
    BatteryModel b;
    EXPECT_DOUBLE_EQ(b.power_w(30.0), b.power_w(-30.0));
    EXPECT_GT(b.power_w(30.0), b.power_w(10.0));
}

TEST(BatteryModel, FractionNeverGoesBelowZero) {
    BatteryModel b(10.0, 0.5);
    for (int i = 0; i < 1000; ++i) b.step(40.0, 1.0);
    EXPECT_DOUBLE_EQ(b.fraction(), 0.0);
}
