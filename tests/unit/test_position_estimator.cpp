#include <gtest/gtest.h>
#include <cmath>
#include "fw_uav/estimation/position_estimator.h"

using namespace fw_uav;

// Helper: home position (roughly Atlanta, GA)
static GeoPosition make_home() {
    return GeoPosition(33.7490, -84.3880, 300.0f);
}

// Helper: GPS data at home position, stationary
static GPSData make_gps_at_home() {
    GPSData gps;
    gps.position = make_home();
    gps.velocity = NEDVelocity(0.0f, 0.0f, 0.0f);
    gps.fix_type = GPSFixType::Fix3D;
    gps.satellites = 10;
    gps.hdop = 1.0f;
    gps.vdop = 1.5f;
    gps.horizontal_accuracy_m = 2.0f;
    gps.vertical_accuracy_m = 4.0f;
    gps.timestamp_us = 0;
    return gps;
}

// Helper: valid baro data
static BarometerData make_baro(float alt_m) {
    BarometerData baro;
    baro.pressure_pa = 101325.0f;
    baro.temperature_c = 25.0f;
    baro.altitude_m = alt_m;
    baro.timestamp_us = 0;
    baro.valid = true;
    return baro;
}

// ---- Construction and configuration ----

TEST(PositionEstimatorTest, DefaultConstruction) {
    PositionEstimator est;
    EXPECT_FALSE(est.initialized());
    EXPECT_FALSE(est.has_home());
}

TEST(PositionEstimatorTest, ConfigConstruction) {
    PositionEstimatorConfig cfg;
    cfg.gps_pos_gain = 0.8f;
    PositionEstimator est(cfg);
    EXPECT_FLOAT_EQ(est.config().gps_pos_gain, 0.8f);
}

TEST(PositionEstimatorTest, SetConfig) {
    PositionEstimator est;
    PositionEstimatorConfig cfg;
    cfg.gps_vel_gain = 0.9f;
    est.set_config(cfg);
    EXPECT_FLOAT_EQ(est.config().gps_vel_gain, 0.9f);
}

// ---- Home position ----

TEST(PositionEstimatorTest, SetHome) {
    PositionEstimator est;
    est.set_home(make_home());
    EXPECT_TRUE(est.has_home());
    EXPECT_NEAR(est.home().latitude_deg, 33.7490, 1e-4);
}

// ---- Reset ----

TEST(PositionEstimatorTest, ResetClearsState) {
    PositionEstimator est;
    est.set_home(make_home());
    est.set_baro_reference(300.0f);
    est.update_gps(make_gps_at_home());

    est.reset();
    EXPECT_FALSE(est.initialized());
    EXPECT_FALSE(est.has_home());
    EXPECT_FLOAT_EQ(est.position().north_m, 0.0f);
    EXPECT_FLOAT_EQ(est.velocity().north_mps, 0.0f);
}

// ---- GPS update ----

TEST(PositionEstimatorTest, GPSRequiresHome) {
    PositionEstimator est;
    auto result = est.update_gps(make_gps_at_home());
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::NoHomePosition);
}

TEST(PositionEstimatorTest, GPSRequiresFix) {
    PositionEstimator est;
    est.set_home(make_home());

    GPSData gps = make_gps_at_home();
    gps.fix_type = GPSFixType::NoFix;
    auto result = est.update_gps(gps);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::GPSNoFix);
}

TEST(PositionEstimatorTest, GPSAtHomeReadsZero) {
    PositionEstimatorConfig cfg;
    cfg.gps_pos_gain = 1.0f;  // Full trust for test clarity
    cfg.gps_vel_gain = 1.0f;
    cfg.gps_alt_gain = 1.0f;
    PositionEstimator est(cfg);
    est.set_home(make_home());

    auto result = est.update_gps(make_gps_at_home());
    EXPECT_TRUE(result.is_ok());
    EXPECT_TRUE(est.initialized());

    EXPECT_NEAR(est.position().north_m, 0.0f, 0.1f);
    EXPECT_NEAR(est.position().east_m, 0.0f, 0.1f);
    EXPECT_NEAR(est.position().down_m, 0.0f, 0.1f);
}

TEST(PositionEstimatorTest, GPSOffsetProducesNED) {
    PositionEstimatorConfig cfg;
    cfg.gps_pos_gain = 1.0f;
    cfg.gps_vel_gain = 1.0f;
    cfg.gps_alt_gain = 1.0f;
    PositionEstimator est(cfg);
    GeoPosition home = make_home();
    est.set_home(home);

    // Move ~111m north (approx 0.001 deg latitude at this location)
    GPSData gps = make_gps_at_home();
    gps.position.latitude_deg = home.latitude_deg + 0.001;
    gps.position.altitude_msl_m = home.altitude_msl_m + 50.0f;  // 50m above home

    auto result = est.update_gps(gps);
    EXPECT_TRUE(result.is_ok());

    EXPECT_GT(est.position().north_m, 100.0f);   // Should be ~111m north
    EXPECT_NEAR(est.position().east_m, 0.0f, 1.0f);
    EXPECT_LT(est.position().down_m, -40.0f);    // 50m above = -50 down
}

TEST(PositionEstimatorTest, GPSVelocityCorrection) {
    PositionEstimatorConfig cfg;
    cfg.gps_vel_gain = 1.0f;
    PositionEstimator est(cfg);
    est.set_home(make_home());

    GPSData gps = make_gps_at_home();
    gps.velocity = NEDVelocity(15.0f, 5.0f, -1.0f);  // 15 m/s north, 5 east, climbing 1 m/s

    est.update_gps(gps);

    EXPECT_NEAR(est.velocity().north_mps, 15.0f, 0.1f);
    EXPECT_NEAR(est.velocity().east_mps, 5.0f, 0.1f);
    EXPECT_NEAR(est.velocity().down_mps, -1.0f, 0.1f);
}

// ---- Barometer update ----

TEST(PositionEstimatorTest, BaroRequiresValid) {
    PositionEstimator est;
    BarometerData baro = make_baro(300.0f);
    baro.valid = false;
    auto result = est.update_baro(baro);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::BarometerReadFailed);
}

TEST(PositionEstimatorTest, BaroWithoutReferenceIsNoop) {
    PositionEstimator est;
    // No baro reference set
    BarometerData baro = make_baro(300.0f);
    auto result = est.update_baro(baro);
    EXPECT_TRUE(result.is_ok());
    // Position should remain at zero
    EXPECT_FLOAT_EQ(est.position().down_m, 0.0f);
}

TEST(PositionEstimatorTest, BaroCorrectionsAltitude) {
    PositionEstimatorConfig cfg;
    cfg.baro_alt_gain = 1.0f;  // Full correction for test
    PositionEstimator est(cfg);
    est.set_baro_reference(300.0f);  // Home baro altitude = 300m

    // Baro reads 350m -> 50m above home -> down = -50
    BarometerData baro = make_baro(350.0f);
    auto result = est.update_baro(baro);
    EXPECT_TRUE(result.is_ok());

    EXPECT_NEAR(est.position().down_m, -50.0f, 0.1f);
    EXPECT_NEAR(est.altitude(), 50.0f, 0.1f);
}

TEST(PositionEstimatorTest, BaroAtReferenceIsZero) {
    PositionEstimatorConfig cfg;
    cfg.baro_alt_gain = 1.0f;
    PositionEstimator est(cfg);
    est.set_baro_reference(300.0f);

    BarometerData baro = make_baro(300.0f);
    est.update_baro(baro);

    EXPECT_NEAR(est.position().down_m, 0.0f, 0.01f);
}

// ---- Predict step ----

TEST(PositionEstimatorTest, PredictWithVelocity) {
    PositionEstimator est;
    est.set_home(make_home());

    // Set an initial velocity via GPS
    PositionEstimatorConfig cfg;
    cfg.gps_vel_gain = 1.0f;
    cfg.gps_pos_gain = 1.0f;
    cfg.gps_alt_gain = 1.0f;
    est.set_config(cfg);

    GPSData gps = make_gps_at_home();
    gps.velocity = NEDVelocity(10.0f, 0.0f, 0.0f);  // 10 m/s north
    est.update_gps(gps);

    // Predict forward 1 second with no acceleration
    est.predict(Vec3f::zero(), 1.0f);

    EXPECT_NEAR(est.position().north_m, 10.0f, 0.5f);
    EXPECT_NEAR(est.position().east_m, 0.0f, 0.1f);
}

TEST(PositionEstimatorTest, PredictWithAcceleration) {
    PositionEstimator est;

    // Start from rest, apply 2 m/s^2 north for 1 second
    float dt = 0.01f;
    Vec3f accel(2.0f, 0.0f, 0.0f);
    for (int i = 0; i < 100; i++) {
        est.predict(accel, dt);
    }

    // After 1s at 2 m/s^2: v = 2 m/s, pos = 0.5*2*1^2 = 1m
    EXPECT_NEAR(est.velocity().north_mps, 2.0f, 0.05f);
    EXPECT_NEAR(est.position().north_m, 1.0f, 0.05f);
}

TEST(PositionEstimatorTest, PredictZeroDtIsNoop) {
    PositionEstimator est;
    est.predict(Vec3f(100.0f, 0.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(est.position().north_m, 0.0f);
    EXPECT_FLOAT_EQ(est.velocity().north_mps, 0.0f);
}

// ---- Complementary filter blending ----

TEST(PositionEstimatorTest, GPSBlendingWithPrediction) {
    PositionEstimatorConfig cfg;
    cfg.gps_pos_gain = 0.5f;
    cfg.gps_vel_gain = 0.5f;
    cfg.gps_alt_gain = 0.5f;
    PositionEstimator est(cfg);
    est.set_home(make_home());

    // First GPS update places us at home
    est.update_gps(make_gps_at_home());
    EXPECT_NEAR(est.position().north_m, 0.0f, 0.1f);

    // Now predict forward as if moving north
    // Then GPS says we're still at home -- the blended result should be
    // between the predicted position and GPS position.
    est.predict(Vec3f::zero(), 0.0f);  // No movement from predict

    // Manually set some predicted position to test blending
    // We do this by predicting with velocity
    GPSData gps_home = make_gps_at_home();
    gps_home.velocity = NEDVelocity(10.0f, 0.0f, 0.0f);
    est.update_gps(gps_home);

    // Predict for 1 second
    est.predict(Vec3f::zero(), 1.0f);
    // Now position should be ~10m north (from velocity)

    // GPS says we are at home (0m north)
    GPSData gps2 = make_gps_at_home();
    gps2.velocity = NEDVelocity(10.0f, 0.0f, 0.0f);
    est.update_gps(gps2);

    // With gain=0.5, position should be blended between predicted and GPS
    EXPECT_NEAR(est.position().north_m, 2.5f, 1.5f);
}

TEST(PositionEstimatorTest, BaroBlendingWithPrediction) {
    PositionEstimatorConfig cfg;
    cfg.baro_alt_gain = 0.5f;
    PositionEstimator est(cfg);
    est.set_baro_reference(100.0f);

    // Baro says 120m -> 20m above home -> down = -20
    BarometerData baro = make_baro(120.0f);
    est.update_baro(baro);

    // With gain=0.5 from zero: 0 + 0.5*(-20 - 0) = -10
    EXPECT_NEAR(est.position().down_m, -10.0f, 0.1f);

    // Second update with same reading: -10 + 0.5*(-20 - (-10)) = -15
    est.update_baro(baro);
    EXPECT_NEAR(est.position().down_m, -15.0f, 0.1f);
}

// ---- GPS + Baro combined ----

TEST(PositionEstimatorTest, GPSAndBaroCombined) {
    PositionEstimatorConfig cfg;
    cfg.gps_pos_gain = 1.0f;
    cfg.gps_vel_gain = 1.0f;
    cfg.gps_alt_gain = 0.0f;   // Disable GPS altitude
    cfg.baro_alt_gain = 1.0f;  // Full baro trust for altitude
    PositionEstimator est(cfg);

    GeoPosition home = make_home();
    est.set_home(home);
    est.set_baro_reference(300.0f);

    // GPS: 100m north, but at home altitude
    GPSData gps = make_gps_at_home();
    gps.position.latitude_deg = home.latitude_deg + 0.001;  // ~111m north
    gps.velocity = NEDVelocity(15.0f, 0.0f, 0.0f);

    est.update_gps(gps);
    EXPECT_GT(est.position().north_m, 100.0f);

    // Baro: 50m above home
    BarometerData baro = make_baro(350.0f);
    est.update_baro(baro);

    EXPECT_NEAR(est.position().down_m, -50.0f, 0.5f);
    EXPECT_NEAR(est.altitude(), 50.0f, 0.5f);

    // Horizontal position should still be ~111m north
    EXPECT_GT(est.position().north_m, 100.0f);
}

// ---- Altitude helper ----

TEST(PositionEstimatorTest, AltitudeIsNegativeDown) {
    PositionEstimatorConfig cfg;
    cfg.baro_alt_gain = 1.0f;
    PositionEstimator est(cfg);
    est.set_baro_reference(0.0f);

    BarometerData baro = make_baro(100.0f);
    est.update_baro(baro);

    EXPECT_NEAR(est.altitude(), 100.0f, 0.1f);
    EXPECT_NEAR(est.position().down_m, -100.0f, 0.1f);
}

// ---- Multiple GPS updates converge ----

TEST(PositionEstimatorTest, RepeatedGPSConverges) {
    PositionEstimatorConfig cfg;
    cfg.gps_pos_gain = 0.3f;
    cfg.gps_alt_gain = 0.3f;
    PositionEstimator est(cfg);
    GeoPosition home = make_home();
    est.set_home(home);

    // GPS consistently says we are 50m north
    GPSData gps = make_gps_at_home();
    double dlat = 50.0 / (math::EARTH_RADIUS_M * math::DEG_TO_RAD);
    gps.position.latitude_deg = home.latitude_deg + dlat;

    for (int i = 0; i < 50; i++) {
        est.update_gps(gps);
    }

    // Should converge close to 50m north
    EXPECT_NEAR(est.position().north_m, 50.0f, 1.0f);
}
