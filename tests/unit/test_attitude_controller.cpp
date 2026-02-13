#include <gtest/gtest.h>
#include "fw_uav/control/attitude_controller.h"

using namespace fw_uav;

class AttitudeControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = AttitudeController(config_);
        controller_.set_sample_rate(400.0f);
    }

    AttitudeControllerConfig config_;
    AttitudeController controller_;
    static constexpr float dt_ = 0.0025f; // 400 Hz
};

TEST_F(AttitudeControllerTest, DefaultConstruction) {
    AttitudeController ctrl;
    EXPECT_EQ(ctrl.mode(), AttitudeControlMode::Angle);
}

TEST_F(AttitudeControllerTest, ConfigConstruction) {
    AttitudeControllerConfig config;
    config.max_roll_rad = 1.0f;
    AttitudeController ctrl(config);
    EXPECT_FLOAT_EQ(ctrl.config().max_roll_rad, 1.0f);
}

TEST_F(AttitudeControllerTest, SetConfig) {
    AttitudeControllerConfig new_config;
    new_config.max_roll_rad = 0.5f;
    controller_.set_config(new_config);
    EXPECT_FLOAT_EQ(controller_.config().max_roll_rad, 0.5f);
}

TEST_F(AttitudeControllerTest, InvalidDtReturnsError) {
    AttitudeSetpoint sp;
    EulerAngles euler;
    Vec3f rates;

    auto result = controller_.update(sp, euler, rates, 0.0f);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::InvalidParameter);

    result = controller_.update(sp, euler, rates, -1.0f);
    EXPECT_TRUE(result.is_error());
}

TEST_F(AttitudeControllerTest, ZeroSetpointZeroState) {
    AttitudeSetpoint sp{0.0f, 0.0f, 0.0f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    auto output = result.value();
    EXPECT_FLOAT_EQ(output.aileron, 0.0f);
    EXPECT_FLOAT_EQ(output.elevator, 0.0f);
    EXPECT_FLOAT_EQ(output.rudder, 0.0f);
}

TEST_F(AttitudeControllerTest, AngleModeRollCorrection) {
    // Aircraft is rolled 10 degrees right, setpoint is wings level
    AttitudeSetpoint sp{0.0f, 0.0f, 0.0f};
    EulerAngles euler{10.0f * math::DEG_TO_RAD, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    // Should command left aileron to correct
    EXPECT_LT(result.value().aileron, 0.0f);
}

TEST_F(AttitudeControllerTest, AngleModePitchCorrection) {
    // Aircraft is pitched up 5 degrees, setpoint is level
    AttitudeSetpoint sp{0.0f, 0.0f, 0.0f};
    EulerAngles euler{0.0f, 5.0f * math::DEG_TO_RAD, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    // Should command nose-down elevator
    EXPECT_LT(result.value().elevator, 0.0f);
}

TEST_F(AttitudeControllerTest, AngleModeYawRateDamping) {
    // Aircraft has yaw rate, should damp it
    AttitudeSetpoint sp{0.0f, 0.0f, 0.0f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.5f};  // Yawing right at 0.5 rad/s

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    // Should command opposing rudder
    EXPECT_LT(result.value().rudder, 0.0f);
}

TEST_F(AttitudeControllerTest, AngleModeSetpointLimiting) {
    // Command more than max roll
    AttitudeSetpoint sp{2.0f, 2.0f, 0.0f};  // Way beyond limits
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    // Output should be finite and bounded
    auto output = result.value();
    EXPECT_GE(output.aileron, -1.0f);
    EXPECT_LE(output.aileron, 1.0f);
    EXPECT_GE(output.elevator, -1.0f);
    EXPECT_LE(output.elevator, 1.0f);
}

TEST_F(AttitudeControllerTest, RateModeDirectControl) {
    controller_.set_mode(AttitudeControlMode::Rate);
    EXPECT_EQ(controller_.mode(), AttitudeControlMode::Rate);

    // Command a roll rate
    AttitudeSetpoint sp{1.0f, 0.0f, 0.0f};  // 1 rad/s roll rate
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};  // Not rotating yet

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    // Should command positive aileron for positive roll rate
    EXPECT_GT(result.value().aileron, 0.0f);
}

TEST_F(AttitudeControllerTest, RateModeRateLimiting) {
    controller_.set_mode(AttitudeControlMode::Rate);

    // Command excessive rate
    AttitudeSetpoint sp{100.0f, 100.0f, 100.0f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());

    // Outputs should be bounded
    auto output = result.value();
    EXPECT_GE(output.aileron, -1.0f);
    EXPECT_LE(output.aileron, 1.0f);
    EXPECT_GE(output.elevator, -1.0f);
    EXPECT_LE(output.elevator, 1.0f);
    EXPECT_GE(output.rudder, -1.0f);
    EXPECT_LE(output.rudder, 1.0f);
}

TEST_F(AttitudeControllerTest, ModeChangeResetsState) {
    // Build up some state in Angle mode
    AttitudeSetpoint sp{0.5f, 0.0f, 0.0f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 10; i++) {
        controller_.update(sp, euler, rates, dt_);
    }

    // Switch to Rate mode - should reset
    controller_.set_mode(AttitudeControlMode::Rate);

    // First update after mode change with zero inputs should produce zero
    sp = {0.0f, 0.0f, 0.0f};
    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());
    EXPECT_FLOAT_EQ(result.value().aileron, 0.0f);
}

TEST_F(AttitudeControllerTest, Reset) {
    // Build up state
    AttitudeSetpoint sp{0.5f, 0.3f, 0.1f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 10; i++) {
        controller_.update(sp, euler, rates, dt_);
    }

    controller_.reset();

    // After reset, zero inputs should produce zero output
    sp = {0.0f, 0.0f, 0.0f};
    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());
    EXPECT_FLOAT_EQ(result.value().aileron, 0.0f);
    EXPECT_FLOAT_EQ(result.value().elevator, 0.0f);
    EXPECT_FLOAT_EQ(result.value().rudder, 0.0f);
}

TEST_F(AttitudeControllerTest, ConvergesToSetpoint) {
    // Simulate a simplified closed loop - verify attitude converges
    float roll = 0.0f;
    float roll_rate = 0.0f;
    float target_roll = 0.3f;  // ~17 degrees

    AttitudeSetpoint sp{target_roll, 0.0f, 0.0f};

    for (int i = 0; i < 2000; i++) {
        EulerAngles euler{roll, 0.0f, 0.0f};
        Vec3f rates{roll_rate, 0.0f, 0.0f};

        auto result = controller_.update(sp, euler, rates, dt_);
        ASSERT_TRUE(result.is_ok());

        // Simple dynamics: torque proportional to aileron
        float torque = result.value().aileron * 10.0f;
        roll_rate += torque * dt_;
        roll_rate *= 0.98f;  // Simple damping
        roll += roll_rate * dt_;
    }

    // Should have converged to target within tolerance
    EXPECT_NEAR(roll, target_roll, 0.1f);  // Within ~6 degrees
}

TEST_F(AttitudeControllerTest, SetSampleRate) {
    // Should not crash
    controller_.set_sample_rate(200.0f);
    controller_.set_sample_rate(1000.0f);

    AttitudeSetpoint sp{0.0f, 0.0f, 0.0f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());
}

TEST_F(AttitudeControllerTest, SameModeSwitchNoReset) {
    // Build up state
    AttitudeSetpoint sp{0.5f, 0.0f, 0.0f};
    EulerAngles euler{0.0f, 0.0f, 0.0f};
    Vec3f rates{0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 10; i++) {
        controller_.update(sp, euler, rates, dt_);
    }

    // Set same mode - should NOT reset
    controller_.set_mode(AttitudeControlMode::Angle);

    // Should still have built-up state
    auto result = controller_.update(sp, euler, rates, dt_);
    ASSERT_TRUE(result.is_ok());
    // With persistent integral, output should be nonzero
    EXPECT_NE(result.value().aileron, 0.0f);
}
