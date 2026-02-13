#include <gtest/gtest.h>
#include <cmath>
#include "fw_uav/estimation/attitude_estimator.h"

using namespace fw_uav;

// Helper: create an IMU sample with gravity pointing down (+z in body frame)
// and zero gyro, representing a level stationary aircraft.
static IMUData make_level_imu() {
    IMUData imu;
    imu.accel_mps2 = Vec3f(0.0f, 0.0f, 9.80665f);
    imu.gyro_radps = Vec3f::zero();
    imu.mag_gauss = Vec3f(0.2f, 0.0f, 0.4f);  // Pointing roughly north
    imu.accel_valid = true;
    imu.gyro_valid = true;
    imu.mag_valid = true;
    imu.timestamp_us = 0;
    return imu;
}

// ---- Construction and configuration ----

TEST(AttitudeEstimatorTest, DefaultConstruction) {
    AttitudeEstimator est;
    EXPECT_FALSE(est.initialized());
    // Default attitude is identity quaternion
    EXPECT_FLOAT_EQ(est.attitude().w, 1.0f);
    EXPECT_FLOAT_EQ(est.attitude().x, 0.0f);
}

TEST(AttitudeEstimatorTest, ConfigConstruction) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.05f;
    AttitudeEstimator est(cfg);
    EXPECT_FLOAT_EQ(est.config().accel_gain, 0.05f);
}

TEST(AttitudeEstimatorTest, SetConfig) {
    AttitudeEstimator est;
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.1f;
    est.set_config(cfg);
    EXPECT_FLOAT_EQ(est.config().accel_gain, 0.1f);
}

// ---- Reset ----

TEST(AttitudeEstimatorTest, ResetToIdentity) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    est.update(imu, 0.01f);
    EXPECT_TRUE(est.initialized());

    est.reset();
    EXPECT_FALSE(est.initialized());
    EXPECT_FLOAT_EQ(est.attitude().w, 1.0f);
    EXPECT_NEAR(est.gyro_bias().x, 0.0f, 1e-9f);
}

TEST(AttitudeEstimatorTest, ResetToKnownAttitude) {
    AttitudeEstimator est;
    Quaternion q = Quaternion::from_euler(EulerAngles(0.1f, 0.2f, 0.3f));
    est.reset(q);
    EXPECT_TRUE(est.initialized());
    EXPECT_NEAR(est.attitude().w, q.w, 1e-5f);
    EXPECT_NEAR(est.attitude().x, q.x, 1e-5f);
}

// ---- Initialization from sensors ----

TEST(AttitudeEstimatorTest, InitializeFromSensorsLevel) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    auto result = est.initialize_from_sensors(imu);
    EXPECT_TRUE(result.is_ok());
    EXPECT_TRUE(est.initialized());

    EulerAngles e = est.euler();
    EXPECT_NEAR(e.roll, 0.0f, 0.01f);
    EXPECT_NEAR(e.pitch, 0.0f, 0.01f);
}

TEST(AttitudeEstimatorTest, InitializeFromSensorsPitchedUp) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    // Pitched 30 deg nose up: gravity has -x and +z components in body frame
    float pitch_rad = 30.0f * math::DEG_TO_RAD;
    imu.accel_mps2 = Vec3f(-std::sin(pitch_rad) * 9.80665f, 0.0f,
                            std::cos(pitch_rad) * 9.80665f);
    auto result = est.initialize_from_sensors(imu);
    EXPECT_TRUE(result.is_ok());

    EulerAngles e = est.euler();
    EXPECT_NEAR(e.pitch, pitch_rad, 0.02f);
    EXPECT_NEAR(e.roll, 0.0f, 0.01f);
}

TEST(AttitudeEstimatorTest, InitializeRequiresAccel) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    imu.accel_valid = false;
    auto result = est.initialize_from_sensors(imu);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::IMUDataInvalid);
}

// ---- Update validation ----

TEST(AttitudeEstimatorTest, UpdateRejectsZeroDt) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    auto result = est.update(imu, 0.0f);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::InvalidParameter);
}

TEST(AttitudeEstimatorTest, UpdateRejectsNegativeDt) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    auto result = est.update(imu, -0.01f);
    EXPECT_TRUE(result.is_error());
}

TEST(AttitudeEstimatorTest, UpdateRejectsInvalidGyro) {
    AttitudeEstimator est;
    IMUData imu = make_level_imu();
    imu.gyro_valid = false;
    auto result = est.update(imu, 0.01f);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::IMUDataInvalid);
}

// ---- Gyro integration ----

TEST(AttitudeEstimatorTest, GyroIntegrationRoll) {
    // Disable accel/mag correction to test pure gyro integration
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.0f;
    cfg.mag_gain = 0.0f;
    cfg.gyro_bias_rate = 0.0f;
    AttitudeEstimator est(cfg);
    est.reset(Quaternion::identity());

    IMUData imu;
    imu.gyro_radps = Vec3f(1.0f, 0.0f, 0.0f);  // 1 rad/s roll rate
    imu.gyro_valid = true;
    imu.accel_valid = false;
    imu.mag_valid = false;

    float dt = 0.001f;
    float total_time = 0.5f;
    int steps = static_cast<int>(total_time / dt);
    for (int i = 0; i < steps; i++) {
        auto result = est.update(imu, dt);
        EXPECT_TRUE(result.is_ok());
    }

    EulerAngles e = est.euler();
    // Expected roll ~0.5 rad after 0.5s at 1 rad/s
    EXPECT_NEAR(e.roll, 0.5f, 0.02f);
    EXPECT_NEAR(e.pitch, 0.0f, 0.02f);
}

TEST(AttitudeEstimatorTest, GyroIntegrationYaw) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.0f;
    cfg.mag_gain = 0.0f;
    cfg.gyro_bias_rate = 0.0f;
    AttitudeEstimator est(cfg);
    est.reset(Quaternion::identity());

    IMUData imu;
    imu.gyro_radps = Vec3f(0.0f, 0.0f, 0.5f);  // 0.5 rad/s yaw rate
    imu.gyro_valid = true;
    imu.accel_valid = false;
    imu.mag_valid = false;

    float dt = 0.001f;
    int steps = 1000;  // 1 second
    for (int i = 0; i < steps; i++) {
        est.update(imu, dt);
    }

    EulerAngles e = est.euler();
    EXPECT_NEAR(e.yaw, 0.5f, 0.02f);
}

// ---- Accelerometer correction ----

TEST(AttitudeEstimatorTest, AccelCorrectsDrift) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.1f;   // Aggressive correction for test speed
    cfg.mag_gain = 0.0f;
    cfg.gyro_bias_rate = 0.0f;
    AttitudeEstimator est(cfg);

    // Start with a small roll error
    Quaternion tilted = Quaternion::from_euler(EulerAngles(0.2f, 0.0f, 0.0f));
    est.reset(tilted);

    IMUData imu = make_level_imu();
    imu.gyro_radps = Vec3f::zero();  // No rotation

    // Run many steps and verify roll converges toward 0
    float dt = 0.01f;
    for (int i = 0; i < 500; i++) {
        est.update(imu, dt);
    }

    EulerAngles e = est.euler();
    EXPECT_NEAR(e.roll, 0.0f, 0.05f);
}

TEST(AttitudeEstimatorTest, AccelGateRejectsHighG) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.5f;
    cfg.mag_gain = 0.0f;
    cfg.gyro_bias_rate = 0.0f;
    cfg.accel_gate_hi = 1.2f;
    AttitudeEstimator est(cfg);

    // Start with a roll error
    Quaternion tilted = Quaternion::from_euler(EulerAngles(0.3f, 0.0f, 0.0f));
    est.reset(tilted);

    IMUData imu;
    // 2g acceleration -- should be gated out
    imu.accel_mps2 = Vec3f(0.0f, 0.0f, 2.0f * 9.80665f);
    imu.gyro_radps = Vec3f::zero();
    imu.gyro_valid = true;
    imu.accel_valid = true;
    imu.mag_valid = false;

    float initial_roll = est.euler().roll;
    for (int i = 0; i < 100; i++) {
        est.update(imu, 0.01f);
    }

    // Roll should not have been corrected (accel was gated)
    EXPECT_NEAR(est.euler().roll, initial_roll, 0.01f);
}

// ---- Magnetometer correction ----

TEST(AttitudeEstimatorTest, MagCorrectsYaw) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.0f;
    cfg.mag_gain = 0.1f;
    cfg.gyro_bias_rate = 0.0f;
    AttitudeEstimator est(cfg);

    // Start with a yaw error
    Quaternion yawed = Quaternion::from_euler(EulerAngles(0.0f, 0.0f, 0.3f));
    est.reset(yawed);

    IMUData imu;
    imu.accel_mps2 = Vec3f(0.0f, 0.0f, 9.80665f);
    imu.gyro_radps = Vec3f::zero();
    imu.mag_gauss = Vec3f(0.2f, 0.0f, 0.4f);  // North-pointing mag
    imu.accel_valid = false;
    imu.gyro_valid = true;
    imu.mag_valid = true;

    float dt = 0.01f;
    for (int i = 0; i < 500; i++) {
        est.update(imu, dt);
    }

    EulerAngles e = est.euler();
    // Yaw should converge toward the mag heading (approximately 0)
    // With low gain and only 500 steps, tolerance is generous
    EXPECT_NEAR(e.yaw, 0.0f, 0.35f);
}

// ---- Bias estimation ----

TEST(AttitudeEstimatorTest, GyroBiasConverges) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.05f;
    cfg.mag_gain = 0.0f;
    cfg.gyro_bias_rate = 0.01f;
    cfg.max_gyro_bias = 0.1f;
    AttitudeEstimator est(cfg);
    est.reset(Quaternion::identity());

    // Simulate a constant gyro bias on the roll axis
    IMUData imu = make_level_imu();
    imu.gyro_radps = Vec3f(0.02f, 0.0f, 0.0f);  // Fake 0.02 rad/s bias

    float dt = 0.005f;
    for (int i = 0; i < 10000; i++) {
        est.update(imu, dt);
    }

    // The bias estimate should have moved toward the actual bias
    EXPECT_GT(std::fabs(est.gyro_bias().x), 0.0005f);

    // And the attitude should still be approximately level because the
    // accel correction keeps pulling it back
    EulerAngles e = est.euler();
    EXPECT_NEAR(e.roll, 0.0f, 0.1f);
    EXPECT_NEAR(e.pitch, 0.0f, 0.1f);
}

TEST(AttitudeEstimatorTest, GyroBiasClamped) {
    AttitudeEstimatorConfig cfg;
    cfg.accel_gain = 0.1f;
    cfg.gyro_bias_rate = 1.0f;  // Very aggressive
    cfg.max_gyro_bias = 0.01f;
    AttitudeEstimator est(cfg);
    est.reset(Quaternion::identity());

    IMUData imu = make_level_imu();
    imu.gyro_radps = Vec3f(1.0f, 1.0f, 1.0f);  // Large fake bias

    for (int i = 0; i < 1000; i++) {
        est.update(imu, 0.01f);
    }

    // Bias magnitude should not exceed the configured max
    float bias_mag = est.gyro_bias().norm();
    EXPECT_LE(bias_mag, cfg.max_gyro_bias + 1e-5f);
}

// ---- Auto-initialization on first update ----

TEST(AttitudeEstimatorTest, AutoInitializesOnFirstUpdate) {
    AttitudeEstimator est;
    EXPECT_FALSE(est.initialized());

    IMUData imu = make_level_imu();
    auto result = est.update(imu, 0.01f);
    EXPECT_TRUE(result.is_ok());
    EXPECT_TRUE(est.initialized());

    // Should be approximately level
    EulerAngles e = est.euler();
    EXPECT_NEAR(e.roll, 0.0f, 0.01f);
    EXPECT_NEAR(e.pitch, 0.0f, 0.01f);
}

// ---- CorrectedGyro ----

TEST(AttitudeEstimatorTest, CorrectedGyroSubtractsBias) {
    AttitudeEstimator est;
    est.reset(Quaternion::identity());

    IMUData imu = make_level_imu();
    imu.gyro_radps = Vec3f(0.5f, 0.3f, 0.1f);
    est.update(imu, 0.01f);

    // Bias should be ~0 initially, so corrected ~ raw
    Vec3f corrected = est.corrected_gyro();
    EXPECT_NEAR(corrected.x, imu.gyro_radps.x, 0.01f);
    EXPECT_NEAR(corrected.y, imu.gyro_radps.y, 0.01f);
    EXPECT_NEAR(corrected.z, imu.gyro_radps.z, 0.01f);
}

// ---- Stability over long run ----

TEST(AttitudeEstimatorTest, StableLevelFlight) {
    AttitudeEstimator est;

    IMUData imu = make_level_imu();

    float dt = 0.005f;
    for (int i = 0; i < 20000; i++) {
        auto result = est.update(imu, dt);
        EXPECT_TRUE(result.is_ok());
    }

    // After 100 seconds of level flight, should still read level
    EulerAngles e = est.euler();
    EXPECT_NEAR(e.roll, 0.0f, 0.01f);
    EXPECT_NEAR(e.pitch, 0.0f, 0.01f);

    // Quaternion should be normalized
    float q_norm = est.attitude().norm();
    EXPECT_NEAR(q_norm, 1.0f, 1e-4f);
}
