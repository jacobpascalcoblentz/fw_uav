#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "fw_uav/safety/failsafe.h"

namespace fw_uav {
namespace {

// Helper to create a default valid AircraftState
AircraftState make_valid_state() {
    AircraftState state{};
    state.position = {100.0f, 50.0f, -80.0f};  // 80m AGL
    state.altitude_agl_m = 80.0f;
    state.airspeed_mps = 20.0f;
    state.in_air = true;
    state.armed = true;
    state.geo_position = {47.0, -122.0, 150.0f};
    return state;
}

// Helper to create a valid RC input
RCInput make_valid_rc() {
    RCInput rc{};
    rc.channel_count = 8;
    rc.failsafe = false;
    rc.frame_lost = false;
    rc.timestamp_us = 1000000;
    return rc;
}

// Helper to create default config
FailsafeConfig make_default_config() {
    FailsafeConfig cfg{};
    cfg.rc_loss_timeout_s = 1.5f;
    cfg.gps_loss_timeout_s = 5.0f;
    cfg.battery_low_voltage = 11.1f;
    cfg.battery_critical_voltage = 10.2f;
    cfg.battery_hysteresis_v = 0.3f;
    return cfg;
}

// ---------------------------------------------------------------------------
// Construction and configuration
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, DefaultConstruction) {
    FailsafeManager mgr;
    EXPECT_FALSE(mgr.any_active());
}

TEST(FailsafeManagerTest, ConfiguredConstruction) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);
    EXPECT_FALSE(mgr.any_active());
    EXPECT_FLOAT_EQ(mgr.config().rc_loss_timeout_s, 1.5f);
}

TEST(FailsafeManagerTest, Reconfigure) {
    FailsafeManager mgr;
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 3.0f;
    mgr.configure(cfg);
    EXPECT_FLOAT_EQ(mgr.config().rc_loss_timeout_s, 3.0f);
}

// ---------------------------------------------------------------------------
// RC loss failsafe
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, RCLoss_NotTriggeredWhenValid) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.failsafe_active);
    EXPECT_FALSE(action.rc_lost);
}

TEST(FailsafeManagerTest, RCLoss_DetectedButNotActiveBeforeTimeout) {
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 1.5f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    RCInput rc{};
    rc.failsafe = true;  // RC lost

    // Run for 1.0 second (50 iterations at 20ms)
    FailsafeAction action;
    for (int i = 0; i < 50; ++i) {
        action = mgr.update(state, rc, 12.6f, 0.02f);
    }

    EXPECT_TRUE(mgr.rc_condition().detected);
    EXPECT_FALSE(action.rc_lost);  // Not yet past 1.5s
}

TEST(FailsafeManagerTest, RCLoss_ActiveAfterTimeout) {
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 1.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    RCInput rc{};
    rc.failsafe = true;

    // Run for 1.1 seconds
    FailsafeAction action;
    for (int i = 0; i < 55; ++i) {
        action = mgr.update(state, rc, 12.6f, 0.02f);
    }

    EXPECT_TRUE(action.failsafe_active);
    EXPECT_TRUE(action.rc_lost);
    EXPECT_EQ(action.recommended_mode, FlightMode::RTL);
}

TEST(FailsafeManagerTest, RCLoss_RecoveryResetsCondition) {
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 0.5f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    RCInput rc_lost{};
    rc_lost.failsafe = true;

    // Trigger failsafe
    for (int i = 0; i < 30; ++i) {
        mgr.update(state, rc_lost, 12.6f, 0.02f);
    }
    EXPECT_TRUE(mgr.rc_condition().active);

    // RC recovers
    auto rc_ok = make_valid_rc();
    auto action = mgr.update(state, rc_ok, 12.6f, 0.02f);
    EXPECT_FALSE(action.rc_lost);
    EXPECT_FALSE(mgr.rc_condition().active);
}

// ---------------------------------------------------------------------------
// GPS loss failsafe
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, GPSLoss_NotTriggeredWhenValid) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.gps_lost);
}

TEST(FailsafeManagerTest, GPSLoss_ActiveAfterTimeout) {
    FailsafeConfig cfg = make_default_config();
    cfg.gps_loss_timeout_s = 2.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position.north_m = std::numeric_limits<float>::quiet_NaN();
    state.position.east_m = std::numeric_limits<float>::quiet_NaN();
    auto rc = make_valid_rc();

    // Run for 2.1 seconds
    FailsafeAction action;
    for (int i = 0; i < 105; ++i) {
        action = mgr.update(state, rc, 12.6f, 0.02f);
    }

    EXPECT_TRUE(action.failsafe_active);
    EXPECT_TRUE(action.gps_lost);
    EXPECT_EQ(action.recommended_mode, FlightMode::Loiter);
}

TEST(FailsafeManagerTest, GPSLoss_RecoveryResetsCondition) {
    FailsafeConfig cfg = make_default_config();
    cfg.gps_loss_timeout_s = 0.5f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position.north_m = std::numeric_limits<float>::quiet_NaN();
    auto rc = make_valid_rc();

    for (int i = 0; i < 30; ++i) {
        mgr.update(state, rc, 12.6f, 0.02f);
    }
    EXPECT_TRUE(mgr.gps_condition().active);

    // GPS recovers
    state.position.north_m = 100.0f;
    state.position.east_m = 50.0f;
    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.gps_lost);
}

// ---------------------------------------------------------------------------
// Battery failsafe
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, Battery_NormalVoltageNoFailsafe) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.battery_low);
    EXPECT_FALSE(action.battery_critical);
}

TEST(FailsafeManagerTest, Battery_LowTriggersRTL) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 10.9f, 0.02f);
    EXPECT_TRUE(action.battery_low);
    EXPECT_FALSE(action.battery_critical);
    EXPECT_TRUE(action.failsafe_active);
}

TEST(FailsafeManagerTest, Battery_CriticalTriggersLand) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 9.8f, 0.02f);
    EXPECT_TRUE(action.battery_low);
    EXPECT_TRUE(action.battery_critical);
    EXPECT_EQ(action.recommended_mode, FlightMode::Land);  // Critical wins by priority
}

TEST(FailsafeManagerTest, Battery_HysteresisPreventsBouncing) {
    FailsafeConfig cfg = make_default_config();
    cfg.battery_low_voltage = 11.1f;
    cfg.battery_hysteresis_v = 0.3f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    // Drop below threshold
    auto action = mgr.update(state, rc, 11.0f, 0.02f);
    EXPECT_TRUE(action.battery_low);

    // Rise to within hysteresis band (11.0 - 11.4)
    action = mgr.update(state, rc, 11.2f, 0.02f);
    EXPECT_TRUE(action.battery_low);  // Still active due to hysteresis

    // Rise above hysteresis band
    action = mgr.update(state, rc, 11.5f, 0.02f);
    EXPECT_FALSE(action.battery_low);  // Cleared
}

// ---------------------------------------------------------------------------
// Geofence failsafe
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, Geofence_DisabledByDefault) {
    FailsafeConfig cfg = make_default_config();
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {9999.0f, 9999.0f, -80.0f};  // Way outside any fence
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.geofence_violated);
}

TEST(FailsafeManagerTest, Geofence_InsideFenceNoViolation) {
    FailsafeConfig cfg = make_default_config();
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 500.0f;
    cfg.geofence.max_altitude_m = 200.0f;
    cfg.geofence.min_altitude_m = 10.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {100.0f, 50.0f, -80.0f};  // Within 500m, 80m AGL
    state.altitude_agl_m = 80.0f;
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.geofence_violated);
}

TEST(FailsafeManagerTest, Geofence_HorizontalViolation) {
    FailsafeConfig cfg = make_default_config();
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 500.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {400.0f, 400.0f, -80.0f};  // ~566m from home
    state.altitude_agl_m = 80.0f;
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_TRUE(action.geofence_violated);
    EXPECT_TRUE(action.failsafe_active);
}

TEST(FailsafeManagerTest, Geofence_AltitudeCeilingViolation) {
    FailsafeConfig cfg = make_default_config();
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 500.0f;
    cfg.geofence.max_altitude_m = 200.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {0.0f, 0.0f, -250.0f};  // 250m AGL, above ceiling
    state.altitude_agl_m = 250.0f;
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_TRUE(action.geofence_violated);
}

TEST(FailsafeManagerTest, Geofence_AltitudeFloorViolation) {
    FailsafeConfig cfg = make_default_config();
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 500.0f;
    cfg.geofence.min_altitude_m = 10.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {0.0f, 0.0f, -5.0f};
    state.altitude_agl_m = 5.0f;
    state.in_air = true;
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_TRUE(action.geofence_violated);
}

TEST(FailsafeManagerTest, Geofence_FloorNotTriggeredOnGround) {
    FailsafeConfig cfg = make_default_config();
    cfg.geofence.enabled = true;
    cfg.geofence.min_altitude_m = 10.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {0.0f, 0.0f, 0.0f};
    state.altitude_agl_m = 0.0f;
    state.in_air = false;  // On ground
    auto rc = make_valid_rc();

    auto action = mgr.update(state, rc, 12.6f, 0.02f);
    EXPECT_FALSE(action.geofence_violated);
}

// ---------------------------------------------------------------------------
// Priority resolution
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, Priority_BatteryCriticalOverridesRCLoss) {
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 0.1f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    RCInput rc_lost{};
    rc_lost.failsafe = true;

    // Trigger RC loss
    for (int i = 0; i < 10; ++i) {
        mgr.update(state, rc_lost, 12.6f, 0.02f);
    }
    EXPECT_TRUE(mgr.rc_condition().active);

    // Now also trigger battery critical
    auto action = mgr.update(state, rc_lost, 9.8f, 0.02f);
    EXPECT_TRUE(action.rc_lost);
    EXPECT_TRUE(action.battery_critical);
    // Battery critical has higher priority
    EXPECT_EQ(action.highest_priority, FailsafePriority::BatteryCritical);
    EXPECT_EQ(action.recommended_mode, FlightMode::Land);
}

TEST(FailsafeManagerTest, Priority_GeofenceOverridesAll) {
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 0.1f;
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 100.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {200.0f, 0.0f, -80.0f};  // Outside fence
    state.altitude_agl_m = 80.0f;
    RCInput rc_lost{};
    rc_lost.failsafe = true;

    // Trigger RC loss + geofence + battery critical
    for (int i = 0; i < 10; ++i) {
        mgr.update(state, rc_lost, 9.8f, 0.02f);
    }

    auto action = mgr.update(state, rc_lost, 9.8f, 0.02f);
    EXPECT_TRUE(action.rc_lost);
    EXPECT_TRUE(action.battery_critical);
    EXPECT_TRUE(action.geofence_violated);
    // Battery critical (5) > Geofence (4) so battery critical wins
    EXPECT_EQ(action.highest_priority, FailsafePriority::BatteryCritical);
    EXPECT_EQ(action.recommended_mode, FlightMode::Land);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, ResetClearsAllConditions) {
    FailsafeConfig cfg = make_default_config();
    cfg.rc_loss_timeout_s = 0.1f;
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 100.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    state.position = {200.0f, 0.0f, -80.0f};
    state.altitude_agl_m = 80.0f;
    RCInput rc_lost{};
    rc_lost.failsafe = true;

    for (int i = 0; i < 10; ++i) {
        mgr.update(state, rc_lost, 9.8f, 0.02f);
    }
    EXPECT_TRUE(mgr.any_active());

    mgr.reset();
    EXPECT_FALSE(mgr.any_active());
    EXPECT_FALSE(mgr.rc_condition().active);
    EXPECT_FALSE(mgr.gps_condition().active);
    EXPECT_FALSE(mgr.battery_low_condition().active);
    EXPECT_FALSE(mgr.battery_critical_condition().active);
    EXPECT_FALSE(mgr.geofence_condition().active);
}

// ---------------------------------------------------------------------------
// No failsafe when all inputs are nominal
// ---------------------------------------------------------------------------

TEST(FailsafeManagerTest, NominalConditions_NoFailsafe) {
    FailsafeConfig cfg = make_default_config();
    cfg.geofence.enabled = true;
    cfg.geofence.radius_m = 500.0f;
    cfg.geofence.max_altitude_m = 200.0f;
    cfg.geofence.min_altitude_m = 10.0f;
    FailsafeManager mgr(cfg);

    auto state = make_valid_state();
    auto rc = make_valid_rc();

    // Run 100 iterations at 50Hz
    FailsafeAction action;
    for (int i = 0; i < 100; ++i) {
        action = mgr.update(state, rc, 12.6f, 0.02f);
    }

    EXPECT_FALSE(action.failsafe_active);
    EXPECT_FALSE(action.rc_lost);
    EXPECT_FALSE(action.gps_lost);
    EXPECT_FALSE(action.battery_low);
    EXPECT_FALSE(action.battery_critical);
    EXPECT_FALSE(action.geofence_violated);
    EXPECT_EQ(action.highest_priority, FailsafePriority::None);
}

} // namespace
} // namespace fw_uav
