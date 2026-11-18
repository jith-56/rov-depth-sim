#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rovsim/pressure_sensor.hpp"

using namespace rovsim;

TEST(PressureSensor, NoiseHasExpectedMeanAndStd) {
    SensorParams p;
    p.noise_std_m = 0.05;
    PressureSensor s(p);
    const int n = 20000;
    double sum = 0.0, sumsq = 0.0;
    for (int i = 0; i < n; ++i) {
        const double e = *s.read(5.0) - 5.0;
        sum += e;
        sumsq += e * e;
    }
    const double mean = sum / n;
    const double std = std::sqrt(sumsq / n - mean * mean);
    EXPECT_NEAR(mean, 0.0, 0.003);
    EXPECT_NEAR(std, 0.05, 0.003);
}

TEST(PressureSensor, BiasShiftsEveryReading) {
    SensorParams p;
    p.noise_std_m = 0.0;
    p.bias_m = 0.15;
    PressureSensor s(p);
    EXPECT_DOUBLE_EQ(*s.read(4.0), 4.15);
}

TEST(PressureSensor, SameSeedGivesIdenticalSequence) {
    PressureSensor a, b;
    for (int i = 0; i < 100; ++i) EXPECT_DOUBLE_EQ(*a.read(1.0), *b.read(1.0));
}

TEST(PressureSensor, DropoutReturnsNothingForExactlyNReads) {
    PressureSensor s;
    s.start_dropout(3);
    EXPECT_TRUE(s.in_dropout());
    EXPECT_FALSE(s.read(2.0).has_value());
    EXPECT_FALSE(s.read(2.0).has_value());
    EXPECT_FALSE(s.read(2.0).has_value());
    EXPECT_FALSE(s.in_dropout());
    EXPECT_TRUE(s.read(2.0).has_value());
}

TEST(PressureSensor, NegativeDropoutLengthIsIgnored) {
    PressureSensor s;
    s.start_dropout(-5);
    EXPECT_TRUE(s.read(1.0).has_value());
}
