#include <gtest/gtest.h>

#include <cmath>

#include "fw_uav/safety/envelope_protection.h"

namespace fw_uav {
namespace {

// Helper to create an aircraft state at normal flight conditions
AircraftState make_normal_state() {
    AircraftState state{};
    state.airspeed_mps = 22.0f;       // Comfortable cruise
    state.altitude_agl_m = 80.0f;     // Well above floor
    state.euler = {0.0f, 0.0f, 0.0f}; // Wings level
    state.in_air = true;
    state.armed = true;
    return state;
}

// Helper to create default config
EnvelopeProtectionConfig make_default_config() {
    EnvelopeProtectionConfig cfg{};
    cfg.min_airspeed_mps = 12.0f;
    cfg.max_airspeed_mps = 40.0f;
    cfg.stall_margin_mps = 2.0f;
    cfg.overspeed_margin_mps = 3.0f;
    cfg.max_bank_angle_rad = 1.0472f;  // ~60 deg
    cfg.min_altitude_m = 15.0f;
    cfg.stall_throttle_boost = 0.8f;
    cfg.overspeed_throttle_limit = 0.0f;
    cfg.stall_max_pitch_up_rad = 0.0f;
    cfg.overspeed_max_pitch_down_rad = 0.0f;
    cfg.alt_floor_min_pitch_rad = 0.0873f;  // ~5 deg
    return cfg;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, DefaultConstruction) {
    EnvelopeProtection ep;
    auto state = make_normal_state();
    EXPECT_FALSE(ep.is_near_stall(state));
}

TEST(EnvelopeProtectionTest, ConfiguredConstruction) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);
    EXPECT_FLOAT_EQ(ep.config().min_airspeed_mps, 12.0f);
}

// ---------------------------------------------------------------------------
// No protections at normal conditions
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, NormalFlight_NoProtections) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    ControlSurfaces surfaces{};
    surfaces.aileron = 0.3f;
    surfaces.elevator = 0.1f;
    surfaces.throttle = 0.6f;

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value(), ProtectionFlags::None);

    // Surfaces should be unchanged
    EXPECT_FLOAT_EQ(surfaces.aileron, 0.3f);
    EXPECT_FLOAT_EQ(surfaces.elevator, 0.1f);
    EXPECT_FLOAT_EQ(surfaces.throttle, 0.6f);
}

// ---------------------------------------------------------------------------
// Stall protection
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, StallProtect_NearStallDetection) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 13.0f;  // Below min(12) + margin(2) = 14
    EXPECT_TRUE(ep.is_near_stall(state));
}

TEST(EnvelopeProtectionTest, StallProtect_NotNearStall) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 20.0f;
    EXPECT_FALSE(ep.is_near_stall(state));
}

TEST(EnvelopeProtectionTest, StallProtect_LimitsElevator) {
    auto cfg = make_default_config();
    cfg.stall_max_pitch_up_rad = 0.0f;  // No nose-up at full stall
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 11.0f;  // Below min (12), full protection

    ControlSurfaces surfaces{};
    surfaces.elevator = 0.8f;   // Trying to pull up hard
    surfaces.throttle = 0.3f;

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::StallProtect));

    // Elevator should be limited (close to 0 at full blend)
    EXPECT_LE(surfaces.elevator, 0.1f);
}

TEST(EnvelopeProtectionTest, StallProtect_BoostsThrottle) {
    auto cfg = make_default_config();
    cfg.stall_throttle_boost = 0.8f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 10.0f;  // Well below stall

    ControlSurfaces surfaces{};
    surfaces.throttle = 0.2f;
    surfaces.elevator = -0.1f;  // Not pulling up, so elevator should not be affected much

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::StallProtect));

    // Throttle should be boosted
    EXPECT_GE(surfaces.throttle, 0.7f);
}

TEST(EnvelopeProtectionTest, StallProtect_GradualRampIn) {
    auto cfg = make_default_config();
    cfg.min_airspeed_mps = 12.0f;
    cfg.stall_margin_mps = 2.0f;
    cfg.stall_throttle_boost = 0.8f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    ControlSurfaces surfaces{};
    surfaces.throttle = 0.1f;
    surfaces.elevator = 0.5f;

    // At threshold (14 m/s) - just entering protection zone
    state.airspeed_mps = 13.9f;
    surfaces.throttle = 0.1f;
    surfaces.elevator = 0.5f;
    ep.apply(surfaces, state);
    float throttle_at_edge = surfaces.throttle;

    // At min (12 m/s) - full protection
    state.airspeed_mps = 12.0f;
    surfaces.throttle = 0.1f;
    surfaces.elevator = 0.5f;
    ep.apply(surfaces, state);
    float throttle_at_min = surfaces.throttle;

    // Throttle should increase as airspeed decreases
    EXPECT_GT(throttle_at_min, throttle_at_edge);
}

TEST(EnvelopeProtectionTest, StallProtect_NoseDownElevatorNotAffected) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 11.0f;

    ControlSurfaces surfaces{};
    surfaces.elevator = -0.5f;  // Pushing nose down (good during stall)
    surfaces.throttle = 0.9f;

    ep.apply(surfaces, state);

    // Nose-down elevator should not be limited during stall protection
    EXPECT_LE(surfaces.elevator, 0.0f);
}

// ---------------------------------------------------------------------------
// Overspeed protection
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, OverspeedProtect_Detection) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 38.0f;  // Above max(40) - margin(3) = 37
    EXPECT_TRUE(ep.is_near_overspeed(state));
}

TEST(EnvelopeProtectionTest, OverspeedProtect_LimitsThrottle) {
    auto cfg = make_default_config();
    cfg.overspeed_throttle_limit = 0.0f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 42.0f;  // Above max

    ControlSurfaces surfaces{};
    surfaces.throttle = 0.9f;
    surfaces.elevator = -0.5f;

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::OverspeedProtect));

    // Throttle should be reduced
    EXPECT_LE(surfaces.throttle, 0.1f);
}

TEST(EnvelopeProtectionTest, OverspeedProtect_LimitsNoseDown) {
    auto cfg = make_default_config();
    cfg.overspeed_max_pitch_down_rad = 0.0f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 42.0f;

    ControlSurfaces surfaces{};
    surfaces.elevator = -0.8f;  // Hard nose-down
    surfaces.throttle = 0.5f;

    ep.apply(surfaces, state);

    // Nose-down should be limited
    EXPECT_GE(surfaces.elevator, -0.1f);
}

// ---------------------------------------------------------------------------
// Overbank protection
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, OverbankProtect_Detection) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.euler.roll = 1.2f;  // ~69 deg, above 60 deg limit
    EXPECT_TRUE(ep.is_overbanked(state));
}

TEST(EnvelopeProtectionTest, OverbankProtect_NotOverbanked) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.euler.roll = 0.5f;  // ~29 deg
    EXPECT_FALSE(ep.is_overbanked(state));
}

TEST(EnvelopeProtectionTest, OverbankProtect_RightBank_LimitsAileron) {
    auto cfg = make_default_config();
    cfg.max_bank_angle_rad = 1.0472f;  // 60 deg
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.euler.roll = 1.2f;  // Over-banked right

    ControlSurfaces surfaces{};
    surfaces.aileron = 0.5f;  // Trying to roll further right

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::OverbankProtect));

    // Aileron should be zeroed (no further right roll)
    EXPECT_LE(surfaces.aileron, 0.0f);
}

TEST(EnvelopeProtectionTest, OverbankProtect_LeftBank_LimitsAileron) {
    auto cfg = make_default_config();
    cfg.max_bank_angle_rad = 1.0472f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.euler.roll = -1.2f;  // Over-banked left

    ControlSurfaces surfaces{};
    surfaces.aileron = -0.5f;  // Trying to roll further left

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::OverbankProtect));

    // Aileron should be zeroed (no further left roll)
    EXPECT_GE(surfaces.aileron, 0.0f);
}

TEST(EnvelopeProtectionTest, OverbankProtect_RecoveryAileronNotLimited) {
    auto cfg = make_default_config();
    cfg.max_bank_angle_rad = 1.0472f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.euler.roll = 1.2f;  // Over-banked right

    ControlSurfaces surfaces{};
    surfaces.aileron = -0.5f;  // Rolling LEFT to recover -- should be allowed

    ep.apply(surfaces, state);

    // Recovery aileron should not be limited
    EXPECT_FLOAT_EQ(surfaces.aileron, -0.5f);
}

// ---------------------------------------------------------------------------
// Altitude floor protection
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, AltFloor_Detection) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.altitude_agl_m = 10.0f;  // Below 15m floor
    EXPECT_TRUE(ep.is_below_floor(state));
}

TEST(EnvelopeProtectionTest, AltFloor_NotBelowFloor) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.altitude_agl_m = 80.0f;
    EXPECT_FALSE(ep.is_below_floor(state));
}

TEST(EnvelopeProtectionTest, AltFloor_PreventsDescentCommand) {
    auto cfg = make_default_config();
    cfg.min_altitude_m = 15.0f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.altitude_agl_m = 5.0f;  // Well below floor

    ControlSurfaces surfaces{};
    surfaces.elevator = -0.5f;  // Trying to descend
    surfaces.throttle = 0.1f;

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::AltFloorProtect));

    // Elevator should be raised to prevent descent
    EXPECT_GT(surfaces.elevator, -0.5f);

    // Throttle should be boosted
    EXPECT_GT(surfaces.throttle, 0.1f);
}

TEST(EnvelopeProtectionTest, AltFloor_HigherAltitudeWeakerProtection) {
    auto cfg = make_default_config();
    cfg.min_altitude_m = 15.0f;
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    ControlSurfaces surfaces1{};
    surfaces1.elevator = -0.5f;
    surfaces1.throttle = 0.1f;

    // Just below floor (14m)
    state.altitude_agl_m = 14.0f;
    ep.apply(surfaces1, state);
    float elevator_at_14 = surfaces1.elevator;
    float throttle_at_14 = surfaces1.throttle;

    // Well below floor (3m)
    ControlSurfaces surfaces2{};
    surfaces2.elevator = -0.5f;
    surfaces2.throttle = 0.1f;
    state.altitude_agl_m = 3.0f;
    ep.apply(surfaces2, state);
    float elevator_at_3 = surfaces2.elevator;
    float throttle_at_3 = surfaces2.throttle;

    // Protection should be stronger at lower altitude
    EXPECT_GT(elevator_at_3, elevator_at_14);
    EXPECT_GT(throttle_at_3, throttle_at_14);
}

// ---------------------------------------------------------------------------
// Multiple protections simultaneously
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, MultipleProtections_StallAndOverbank) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 11.0f;   // Stall
    state.euler.roll = 1.3f;      // Over-banked

    ControlSurfaces surfaces{};
    surfaces.elevator = 0.8f;
    surfaces.aileron = 0.5f;
    surfaces.throttle = 0.2f;

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::StallProtect));
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::OverbankProtect));
}

TEST(EnvelopeProtectionTest, MultipleProtections_OverspeedAndAltFloor) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 42.0f;    // Overspeed
    state.altitude_agl_m = 5.0f;   // Below floor

    ControlSurfaces surfaces{};
    surfaces.elevator = -0.8f;
    surfaces.throttle = 0.9f;

    auto result = ep.apply(surfaces, state);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::OverspeedProtect));
    EXPECT_TRUE(has_flag(result.value(), ProtectionFlags::AltFloorProtect));
}

// ---------------------------------------------------------------------------
// Output clamping
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, OutputsClampedToValidRange) {
    auto cfg = make_default_config();
    EnvelopeProtection ep(cfg);

    auto state = make_normal_state();
    state.airspeed_mps = 8.0f;  // Deep stall

    ControlSurfaces surfaces{};
    surfaces.elevator = 0.95f;
    surfaces.throttle = 0.0f;

    ep.apply(surfaces, state);

    EXPECT_GE(surfaces.throttle, 0.0f);
    EXPECT_LE(surfaces.throttle, 1.0f);
    EXPECT_GE(surfaces.elevator, -1.0f);
    EXPECT_LE(surfaces.elevator, 1.0f);
    EXPECT_GE(surfaces.aileron, -1.0f);
    EXPECT_LE(surfaces.aileron, 1.0f);
}

// ---------------------------------------------------------------------------
// Flag utility
// ---------------------------------------------------------------------------

TEST(EnvelopeProtectionTest, ProtectionFlags_BitwiseOr) {
    ProtectionFlags flags = ProtectionFlags::StallProtect | ProtectionFlags::OverbankProtect;
    EXPECT_TRUE(has_flag(flags, ProtectionFlags::StallProtect));
    EXPECT_TRUE(has_flag(flags, ProtectionFlags::OverbankProtect));
    EXPECT_FALSE(has_flag(flags, ProtectionFlags::OverspeedProtect));
    EXPECT_FALSE(has_flag(flags, ProtectionFlags::AltFloorProtect));
}

TEST(EnvelopeProtectionTest, ProtectionFlags_None) {
    ProtectionFlags flags = ProtectionFlags::None;
    EXPECT_FALSE(has_flag(flags, ProtectionFlags::StallProtect));
    EXPECT_FALSE(has_flag(flags, ProtectionFlags::OverspeedProtect));
    EXPECT_FALSE(has_flag(flags, ProtectionFlags::OverbankProtect));
    EXPECT_FALSE(has_flag(flags, ProtectionFlags::AltFloorProtect));
}

} // namespace
} // namespace fw_uav
