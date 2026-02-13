#include <gtest/gtest.h>
#include "fw_uav/control/tecs.h"

using namespace fw_uav;

class TECSTest : public ::testing::Test {
protected:
    void SetUp() override {
        tecs_ = TECSController(config_);
    }

    TECSConfig config_;
    TECSController tecs_;
    static constexpr float dt_ = 0.02f; // 50 Hz
};

TEST_F(TECSTest, DefaultConstruction) {
    TECSController ctrl;
    EXPECT_FALSE(ctrl.initialized());
}

TEST_F(TECSTest, ConfigConstruction) {
    TECSConfig config;
    config.cruise_throttle = 0.6f;
    TECSController ctrl(config);
    EXPECT_FLOAT_EQ(ctrl.config().cruise_throttle, 0.6f);
}

TEST_F(TECSTest, SetConfig) {
    TECSConfig new_config;
    new_config.min_airspeed_mps = 15.0f;
    tecs_.set_config(new_config);
    EXPECT_FLOAT_EQ(tecs_.config().min_airspeed_mps, 15.0f);
}

TEST_F(TECSTest, InvalidDtReturnsDefault) {
    auto output = tecs_.update(20.0f, 100.0f, 20.0f, 100.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(output.throttle, 0.0f);

    output = tecs_.update(20.0f, 100.0f, 20.0f, 100.0f, 0.0f, -1.0f);
    EXPECT_FLOAT_EQ(output.throttle, 0.0f);
}

TEST_F(TECSTest, SteadyStateCruise) {
    // At setpoint: should produce cruise throttle and near-zero pitch
    float airspeed = 20.0f;
    float altitude = 100.0f;

    TECSOutput output;
    for (int i = 0; i < 100; i++) {
        output = tecs_.update(airspeed, altitude, airspeed, altitude, 0.0f, dt_);
    }

    // Should converge near cruise throttle
    EXPECT_NEAR(output.throttle, config_.cruise_throttle, 0.15f);
    // Pitch should be near zero (no altitude or speed correction needed)
    EXPECT_NEAR(output.pitch_ref_rad, 0.0f, 0.15f);
}

TEST_F(TECSTest, ClimbDemand) {
    // Below altitude setpoint: should increase throttle and pitch up
    float airspeed = 20.0f;
    float current_altitude = 90.0f;
    float target_altitude = 110.0f;

    auto output = tecs_.update(airspeed, target_altitude, airspeed, current_altitude, 0.0f, dt_);
    // First update initializes, do a second
    output = tecs_.update(airspeed, target_altitude, airspeed, current_altitude, 0.0f, dt_);

    // Should want more throttle than cruise (need more total energy)
    EXPECT_GT(output.throttle, config_.cruise_throttle);
    // Should pitch up (trade to potential energy)
    EXPECT_GT(output.pitch_ref_rad, 0.0f);
}

TEST_F(TECSTest, DescentDemand) {
    // Above altitude setpoint: should decrease throttle
    float airspeed = 20.0f;
    float current_altitude = 110.0f;
    float target_altitude = 90.0f;

    auto output = tecs_.update(airspeed, target_altitude, airspeed, current_altitude, 0.0f, dt_);
    output = tecs_.update(airspeed, target_altitude, airspeed, current_altitude, 0.0f, dt_);

    // Should want less throttle than cruise
    EXPECT_LT(output.throttle, config_.cruise_throttle);
    // Should pitch down
    EXPECT_LT(output.pitch_ref_rad, 0.0f);
}

TEST_F(TECSTest, SpeedIncreaseDemand) {
    // Below airspeed setpoint, at altitude
    float current_airspeed = 15.0f;
    float target_airspeed = 25.0f;
    float altitude = 100.0f;

    auto output = tecs_.update(target_airspeed, altitude, current_airspeed, altitude, 0.0f, dt_);
    output = tecs_.update(target_airspeed, altitude, current_airspeed, altitude, 0.0f, dt_);

    // Should want more throttle (more total energy for speed)
    EXPECT_GT(output.throttle, config_.cruise_throttle);
}

TEST_F(TECSTest, ThrottleLimits) {
    // Extreme demands should still be bounded
    auto output = tecs_.update(30.0f, 500.0f, 10.0f, 50.0f, 0.0f, dt_);
    output = tecs_.update(30.0f, 500.0f, 10.0f, 50.0f, 0.0f, dt_);

    EXPECT_GE(output.throttle, config_.min_throttle);
    EXPECT_LE(output.throttle, config_.max_throttle);
}

TEST_F(TECSTest, PitchLimits) {
    // Extreme demands should still produce bounded pitch
    for (int i = 0; i < 50; i++) {
        auto output = tecs_.update(30.0f, 500.0f, 10.0f, 50.0f, 0.0f, dt_);
        EXPECT_GE(output.pitch_ref_rad, config_.min_pitch_rad);
        EXPECT_LE(output.pitch_ref_rad, config_.max_pitch_rad);
    }
}

TEST_F(TECSTest, AirspeedClamping) {
    // Setpoint below minimum should be clamped
    auto output = tecs_.update(5.0f, 100.0f, 20.0f, 100.0f, 0.0f, dt_);
    output = tecs_.update(5.0f, 100.0f, 20.0f, 100.0f, 0.0f, dt_);

    // Should still produce valid output
    EXPECT_GE(output.throttle, config_.min_throttle);
    EXPECT_LE(output.throttle, config_.max_throttle);
}

TEST_F(TECSTest, Reset) {
    // Build up state
    for (int i = 0; i < 20; i++) {
        tecs_.update(25.0f, 150.0f, 15.0f, 100.0f, 0.0f, dt_);
    }
    EXPECT_TRUE(tecs_.initialized());
    EXPECT_NE(tecs_.ste_integral(), 0.0f);

    tecs_.reset();
    EXPECT_FALSE(tecs_.initialized());
    EXPECT_FLOAT_EQ(tecs_.ste_integral(), 0.0f);
    EXPECT_FLOAT_EQ(tecs_.sbe_integral(), 0.0f);
}

TEST_F(TECSTest, ResetIntegrals) {
    for (int i = 0; i < 20; i++) {
        tecs_.update(25.0f, 150.0f, 15.0f, 100.0f, 0.0f, dt_);
    }

    tecs_.reset_integrals();
    EXPECT_FLOAT_EQ(tecs_.ste_integral(), 0.0f);
    EXPECT_FLOAT_EQ(tecs_.sbe_integral(), 0.0f);
    // Should still be initialized
    EXPECT_TRUE(tecs_.initialized());
}

TEST_F(TECSTest, DiagnosticOutput) {
    auto output = tecs_.update(20.0f, 100.0f, 15.0f, 90.0f, 0.0f, dt_);

    // Diagnostics should reflect errors
    EXPECT_GT(output.speed_error_mps, 0.0f);      // Want more speed
    EXPECT_GT(output.altitude_error_m, 0.0f);      // Want more altitude
}

TEST_F(TECSTest, FactoryConfigs) {
    auto hp = TECSConfig::HighPerformance();
    EXPECT_GT(hp.max_climb_rate_mps, config_.max_climb_rate_mps);
    EXPECT_LT(hp.time_constant_s, config_.time_constant_s);

    auto cons = TECSConfig::Conservative();
    EXPECT_LT(cons.max_climb_rate_mps, config_.max_climb_rate_mps);
    EXPECT_GT(cons.time_constant_s, config_.time_constant_s);
}
