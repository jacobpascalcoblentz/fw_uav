#include <gtest/gtest.h>
#include "fw_uav/control/pid.h"

using namespace fw_uav;

TEST(PIDConfigTest, StaticConstructors) {
    auto p = PIDConfig::P(1.0f);
    EXPECT_FLOAT_EQ(p.kp, 1.0f);
    EXPECT_FLOAT_EQ(p.ki, 0.0f);
    EXPECT_FLOAT_EQ(p.kd, 0.0f);

    auto pi = PIDConfig::PI(1.0f, 0.5f);
    EXPECT_FLOAT_EQ(pi.kp, 1.0f);
    EXPECT_FLOAT_EQ(pi.ki, 0.5f);
    EXPECT_FLOAT_EQ(pi.kd, 0.0f);

    auto pid = PIDConfig::PID(1.0f, 0.5f, 0.1f);
    EXPECT_FLOAT_EQ(pid.kp, 1.0f);
    EXPECT_FLOAT_EQ(pid.ki, 0.5f);
    EXPECT_FLOAT_EQ(pid.kd, 0.1f);
}

TEST(PIDControllerTest, DefaultConstruction) {
    PIDController pid;
    EXPECT_FALSE(pid.initialized());
}

TEST(PIDControllerTest, ProportionalOnly) {
    PIDController pid(PIDConfig::P(2.0f));
    float output = pid.update(10.0f, 5.0f, 0.01f);
    // Error = 10 - 5 = 5, output = 2.0 * 5 = 10, clamped to 1.0
    EXPECT_FLOAT_EQ(output, 1.0f);

    pid.reset();
    output = pid.update(5.0f, 5.0f, 0.01f);
    EXPECT_FLOAT_EQ(output, 0.0f);  // No error
}

TEST(PIDControllerTest, IntegralBuildup) {
    PIDController pid(PIDConfig::PI(0.0f, 1.0f));

    // Apply constant error over time
    float output = 0.0f;
    for (int i = 0; i < 10; i++) {
        output = pid.update(1.0f, 0.0f, 0.1f);  // Error of 1.0
    }
    // Integral = 1.0 * 0.1 * 10 = 1.0, output = 1.0 * 1.0 = 1.0 (clamped)
    EXPECT_NEAR(output, 1.0f, 0.01f);
}

TEST(PIDControllerTest, OutputLimits) {
    PIDConfig config = PIDConfig::P(100.0f);
    config.output_min = -0.5f;
    config.output_max = 0.5f;
    PIDController pid(config);

    float output = pid.update(10.0f, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(output, 0.5f);  // Clamped to max

    output = pid.update(-10.0f, 0.0f, 0.01f);
    EXPECT_FLOAT_EQ(output, -0.5f);  // Clamped to min
}

TEST(PIDControllerTest, Reset) {
    PIDController pid(PIDConfig::PI(1.0f, 1.0f));

    // Build up some integral
    for (int i = 0; i < 10; i++) {
        pid.update(1.0f, 0.0f, 0.1f);
    }
    EXPECT_GT(pid.integral(), 0.0f);

    pid.reset();
    EXPECT_FLOAT_EQ(pid.integral(), 0.0f);
    EXPECT_FALSE(pid.initialized());
}

TEST(PIDControllerTest, ResetIntegralOnly) {
    PIDController pid(PIDConfig::PI(1.0f, 1.0f));

    for (int i = 0; i < 10; i++) {
        pid.update(1.0f, 0.0f, 0.1f);
    }
    EXPECT_TRUE(pid.initialized());

    pid.reset_integral();
    EXPECT_FLOAT_EQ(pid.integral(), 0.0f);
    EXPECT_TRUE(pid.initialized());  // Still initialized
}

TEST(PIDControllerTest, ZeroDtReturnsLastOutput) {
    PIDController pid(PIDConfig::P(1.0f));
    float first = pid.update(10.0f, 5.0f, 0.01f);
    float second = pid.update(10.0f, 5.0f, 0.0f);  // Zero dt
    EXPECT_FLOAT_EQ(first, second);
}

TEST(PIDControllerTest, Feedforward) {
    PIDConfig config = PIDConfig::P(0.0f);  // No P gain
    config.ff = 2.0f;
    PIDController pid(config);

    float output = pid.update(0.0f, 0.0f, 0.01f, 0.25f);  // feedforward = 0.25
    EXPECT_FLOAT_EQ(output, 0.5f);  // ff * feedforward = 2.0 * 0.25
}

TEST(PIDControllerTest, SetConfig) {
    PIDController pid(PIDConfig::P(1.0f));

    PIDConfig new_config = PIDConfig::PI(2.0f, 0.5f);
    pid.set_config(new_config);

    EXPECT_FLOAT_EQ(pid.config().kp, 2.0f);
    EXPECT_FLOAT_EQ(pid.config().ki, 0.5f);
}

TEST(CascadedPIDTest, BasicOperation) {
    PIDConfig outer = PIDConfig::P(2.0f);
    PIDConfig inner = PIDConfig::P(1.0f);
    CascadedPID cascaded(outer, inner);

    // Outer: angle error * 2.0 = rate setpoint
    // Inner: rate error * 1.0 = output
    float output = cascaded.update(
        1.0f,   // angle setpoint
        0.0f,   // angle measurement
        0.0f,   // rate measurement
        0.01f   // dt
    );
    // Outer: (1.0 - 0.0) * 2.0 = 2.0 rate setpoint (clamped to 1.0)
    // Inner: (1.0 - 0.0) * 1.0 = 1.0
    EXPECT_FLOAT_EQ(output, 1.0f);
}

TEST(CascadedPIDTest, Reset) {
    CascadedPID cascaded(PIDConfig::PI(1.0f, 0.5f), PIDConfig::PI(1.0f, 0.5f));

    // Build up state
    for (int i = 0; i < 10; i++) {
        cascaded.update(1.0f, 0.0f, 0.0f, 0.1f);
    }

    cascaded.reset();
    EXPECT_FLOAT_EQ(cascaded.outer().integral(), 0.0f);
    EXPECT_FLOAT_EQ(cascaded.inner().integral(), 0.0f);
}

TEST(CascadedPIDTest, SetSampleRate) {
    CascadedPID cascaded(PIDConfig::P(1.0f), PIDConfig::P(1.0f));
    cascaded.set_sample_rate(200.0f);
    // Just verify it doesn't crash - actual filter behavior is internal
}
