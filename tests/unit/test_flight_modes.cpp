#include <gtest/gtest.h>
#include "fw_uav/modes/flight_mode_manager.h"
#include "fw_uav/modes/mode_manual.h"
#include "fw_uav/modes/mode_stabilize.h"
#include "fw_uav/modes/mode_fbw.h"
#include "fw_uav/modes/mode_auto.h"

using namespace fw_uav;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static AircraftState make_level_state() {
    AircraftState state;
    state.attitude = Quaternion::identity();
    state.euler = EulerAngles(0.0f, 0.0f, 0.0f);
    state.angular_rate_radps = Vec3f::zero();
    state.position = NEDPosition(0.0f, 0.0f, -100.0f);
    state.velocity = NEDVelocity(18.0f, 0.0f, 0.0f);
    state.airspeed_mps = 18.0f;
    state.altitude_msl_m = 100.0f;
    state.altitude_agl_m = 100.0f;
    state.armed = true;
    state.in_air = true;
    return state;
}

static RCInput make_centered_rc() {
    RCInput rc;
    rc.channel_count = 8;
    rc.failsafe = false;
    rc.frame_lost = false;
    for (int i = 0; i < RCInput::MAX_CHANNELS; ++i) {
        rc.channels[i] = 0.0f;
    }
    rc.channels[2] = 0.5f;  // Throttle at 50%
    return rc;
}

// ---------------------------------------------------------------------------
// FlightModeManager tests
// ---------------------------------------------------------------------------

class FlightModeManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = make_level_state();
        rc_ = make_centered_rc();
    }

    FlightModeManager manager_;
    ModeManual manual_;
    AircraftState state_;
    RCInput rc_;
    static constexpr float dt_ = 0.01f;
};

TEST_F(FlightModeManagerTest, DefaultHasNoActiveMode) {
    auto result = manager_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::ControlNotInitialized);
}

TEST_F(FlightModeManagerTest, RegisterMode) {
    EXPECT_TRUE(manager_.register_mode(&manual_));
    EXPECT_TRUE(manager_.has_mode(FlightMode::Manual));
    EXPECT_FALSE(manager_.has_mode(FlightMode::Stabilize));
}

TEST_F(FlightModeManagerTest, RegisterNullFails) {
    EXPECT_FALSE(manager_.register_mode(nullptr));
}

TEST_F(FlightModeManagerTest, RegisterDuplicateFails) {
    ModeManual manual2;
    EXPECT_TRUE(manager_.register_mode(&manual_));
    EXPECT_FALSE(manager_.register_mode(&manual2));
}

TEST_F(FlightModeManagerTest, RequestUnregisteredModeFails) {
    auto result = manager_.request_mode(FlightMode::Stabilize, state_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::ModeChangeRejected);
}

TEST_F(FlightModeManagerTest, RequestModeSuccess) {
    manager_.register_mode(&manual_);
    auto result = manager_.request_mode(FlightMode::Manual, state_);
    EXPECT_TRUE(result.is_ok());
    EXPECT_EQ(manager_.current_mode(), FlightMode::Manual);
    EXPECT_EQ(manager_.current_mode_ptr(), &manual_);
}

TEST_F(FlightModeManagerTest, UpdateDelegatesToActiveMode) {
    manager_.register_mode(&manual_);
    manager_.request_mode(FlightMode::Manual, state_);

    auto result = manager_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_ok());
}

TEST_F(FlightModeManagerTest, ModeTransitionCallsExitAndEnter) {
    // We verify indirectly: switching between manual and stabilize works
    AttitudeController attitude_ctrl;
    ModeStabilize stabilize(&attitude_ctrl);

    manager_.register_mode(&manual_);
    manager_.register_mode(&stabilize);

    manager_.request_mode(FlightMode::Manual, state_);
    EXPECT_EQ(manager_.current_mode(), FlightMode::Manual);

    manager_.request_mode(FlightMode::Stabilize, state_);
    EXPECT_EQ(manager_.current_mode(), FlightMode::Stabilize);

    manager_.request_mode(FlightMode::Manual, state_);
    EXPECT_EQ(manager_.current_mode(), FlightMode::Manual);
}

// ---------------------------------------------------------------------------
// ModeManual tests
// ---------------------------------------------------------------------------

class ModeManualTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = make_level_state();
        rc_ = make_centered_rc();
    }

    ModeManual manual_;
    AircraftState state_;
    RCInput rc_;
    static constexpr float dt_ = 0.01f;
};

TEST_F(ModeManualTest, IdAndName) {
    EXPECT_EQ(manual_.id(), FlightMode::Manual);
    EXPECT_STREQ(manual_.name(), "Manual");
}

TEST_F(ModeManualTest, CanAlwaysEnter) {
    EXPECT_TRUE(manual_.can_enter(state_));
    state_.airspeed_mps = 0.0f;
    EXPECT_TRUE(manual_.can_enter(state_));
}

TEST_F(ModeManualTest, PassthroughCentered) {
    auto result = manual_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    const auto& s = result.value();
    EXPECT_FLOAT_EQ(s.aileron, 0.0f);
    EXPECT_FLOAT_EQ(s.elevator, 0.0f);
    EXPECT_FLOAT_EQ(s.throttle, 0.5f);
    EXPECT_FLOAT_EQ(s.rudder, 0.0f);
}

TEST_F(ModeManualTest, PassthroughFullDeflection) {
    rc_.channels[0] = 1.0f;   // Roll right
    rc_.channels[1] = -1.0f;  // Pitch down
    rc_.channels[2] = 1.0f;   // Full throttle
    rc_.channels[3] = 0.5f;   // Half rudder right

    auto result = manual_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    const auto& s = result.value();
    EXPECT_FLOAT_EQ(s.aileron, 1.0f);
    EXPECT_FLOAT_EQ(s.elevator, -1.0f);
    EXPECT_FLOAT_EQ(s.throttle, 1.0f);
    EXPECT_FLOAT_EQ(s.rudder, 0.5f);
}

TEST_F(ModeManualTest, FailsafeReturnsError) {
    rc_.failsafe = true;
    auto result = manual_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::RCLost);
}

TEST_F(ModeManualTest, FrameLostReturnsError) {
    rc_.frame_lost = true;
    auto result = manual_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::RCLost);
}

// ---------------------------------------------------------------------------
// ModeStabilize tests
// ---------------------------------------------------------------------------

class ModeStabilizeTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = make_level_state();
        rc_ = make_centered_rc();
        attitude_ctrl_ = AttitudeController(AttitudeControllerConfig{});
        attitude_ctrl_.set_sample_rate(100.0f);
        stabilize_ = ModeStabilize(&attitude_ctrl_);
    }

    AttitudeController attitude_ctrl_;
    ModeStabilize stabilize_;
    AircraftState state_;
    RCInput rc_;
    static constexpr float dt_ = 0.01f;
};

TEST_F(ModeStabilizeTest, IdAndName) {
    EXPECT_EQ(stabilize_.id(), FlightMode::Stabilize);
    EXPECT_STREQ(stabilize_.name(), "Stabilize");
}

TEST_F(ModeStabilizeTest, CanEnterWithController) {
    EXPECT_TRUE(stabilize_.can_enter(state_));
}

TEST_F(ModeStabilizeTest, CannotEnterWithoutController) {
    ModeStabilize no_ctrl;
    EXPECT_FALSE(no_ctrl.can_enter(state_));
}

TEST_F(ModeStabilizeTest, EnterResetsController) {
    // Just verify enter() doesn't crash
    stabilize_.enter();
    EXPECT_EQ(attitude_ctrl_.mode(), AttitudeControlMode::Angle);
}

TEST_F(ModeStabilizeTest, CenteredSticksLevelFlight) {
    stabilize_.enter();
    auto result = stabilize_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());

    const auto& s = result.value();
    // Level state + centered sticks => near-zero corrections
    EXPECT_NEAR(s.aileron, 0.0f, 0.1f);
    EXPECT_NEAR(s.elevator, 0.0f, 0.1f);
    // Throttle is passthrough
    EXPECT_FLOAT_EQ(s.throttle, 0.5f);
}

TEST_F(ModeStabilizeTest, ThrottlePassthrough) {
    stabilize_.enter();
    rc_.channels[2] = 0.8f;
    auto result = stabilize_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    EXPECT_FLOAT_EQ(result.value().throttle, 0.8f);
}

TEST_F(ModeStabilizeTest, FailsafeReturnsError) {
    stabilize_.enter();
    rc_.failsafe = true;
    auto result = stabilize_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::RCLost);
}

TEST_F(ModeStabilizeTest, NoControllerReturnsError) {
    ModeStabilize no_ctrl;
    no_ctrl.enter();
    auto result = no_ctrl.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::ControlNotInitialized);
}

TEST_F(ModeStabilizeTest, RollStickCommandsBankAngle) {
    stabilize_.enter();

    // Command right roll
    rc_.channels[0] = 0.5f;

    // Run a few iterations to let controller respond
    for (int i = 0; i < 10; ++i) {
        auto result = stabilize_.update(state_, rc_, dt_);
        ASSERT_TRUE(result.is_ok());
    }

    // With a right roll command on a level aircraft, aileron should deflect positive
    auto result = stabilize_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    EXPECT_GT(result.value().aileron, 0.0f);
}

// ---------------------------------------------------------------------------
// ModeFBW tests
// ---------------------------------------------------------------------------

class ModeFBWTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = make_level_state();
        rc_ = make_centered_rc();
        attitude_ctrl_ = AttitudeController(AttitudeControllerConfig{});
        attitude_ctrl_.set_sample_rate(100.0f);
        tecs_ctrl_ = TECSController(TECSConfig::Default());
        fbw_ = ModeFBW(&attitude_ctrl_, &tecs_ctrl_);
    }

    AttitudeController attitude_ctrl_;
    TECSController tecs_ctrl_;
    ModeFBW fbw_;
    AircraftState state_;
    RCInput rc_;
    static constexpr float dt_ = 0.01f;
};

TEST_F(ModeFBWTest, IdAndName) {
    EXPECT_EQ(fbw_.id(), FlightMode::FlyByWire);
    EXPECT_STREQ(fbw_.name(), "FlyByWire");
}

TEST_F(ModeFBWTest, CanEnterWithControllersAndAirspeed) {
    EXPECT_TRUE(fbw_.can_enter(state_));
}

TEST_F(ModeFBWTest, CannotEnterWithoutAirspeed) {
    state_.airspeed_mps = 0.0f;
    EXPECT_FALSE(fbw_.can_enter(state_));
}

TEST_F(ModeFBWTest, CannotEnterWithoutTECS) {
    ModeFBW no_tecs(&attitude_ctrl_, nullptr);
    EXPECT_FALSE(no_tecs.can_enter(state_));
}

TEST_F(ModeFBWTest, EnterInitializesAltitudeSetpoint) {
    fbw_.enter();
    // First update should initialize altitude setpoint to current
    auto result = fbw_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    EXPECT_FLOAT_EQ(fbw_.altitude_setpoint(), state_.altitude_msl_m);
}

TEST_F(ModeFBWTest, SteadyStateLevelFlight) {
    fbw_.enter();

    // Run a few iterations at steady state
    for (int i = 0; i < 20; ++i) {
        auto result = fbw_.update(state_, rc_, dt_);
        ASSERT_TRUE(result.is_ok());
    }

    auto result = fbw_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    const auto& s = result.value();

    // All outputs should be in valid range
    EXPECT_GE(s.aileron, -1.0f);
    EXPECT_LE(s.aileron, 1.0f);
    EXPECT_GE(s.elevator, -1.0f);
    EXPECT_LE(s.elevator, 1.0f);
    EXPECT_GE(s.throttle, 0.0f);
    EXPECT_LE(s.throttle, 1.0f);
}

TEST_F(ModeFBWTest, FailsafeReturnsError) {
    fbw_.enter();
    rc_.failsafe = true;
    auto result = fbw_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::RCLost);
}

TEST_F(ModeFBWTest, ThrottleStickChangesAirspeedSetpoint) {
    fbw_.enter();

    // Low throttle -> near min airspeed
    rc_.channels[2] = 0.0f;
    fbw_.update(state_, rc_, dt_);
    float low_sp = fbw_.airspeed_setpoint();

    // High throttle -> near max airspeed
    rc_.channels[2] = 1.0f;
    fbw_.update(state_, rc_, dt_);
    float high_sp = fbw_.airspeed_setpoint();

    EXPECT_LT(low_sp, high_sp);
}

// ---------------------------------------------------------------------------
// ModeAuto tests
// ---------------------------------------------------------------------------

class ModeAutoTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = make_level_state();
        rc_ = make_centered_rc();
        attitude_ctrl_ = AttitudeController(AttitudeControllerConfig{});
        attitude_ctrl_.set_sample_rate(100.0f);
        tecs_ctrl_ = TECSController(TECSConfig::Default());
        l1_ctrl_ = L1Controller(L1Config{});
        auto_mode_ = ModeAuto(&attitude_ctrl_, &tecs_ctrl_, &l1_ctrl_);
    }

    AttitudeController attitude_ctrl_;
    TECSController tecs_ctrl_;
    L1Controller l1_ctrl_;
    ModeAuto auto_mode_;
    AircraftState state_;
    RCInput rc_;
    static constexpr float dt_ = 0.01f;
};

TEST_F(ModeAutoTest, IdAndName) {
    EXPECT_EQ(auto_mode_.id(), FlightMode::Auto);
    EXPECT_STREQ(auto_mode_.name(), "Auto");
}

TEST_F(ModeAutoTest, CannotEnterWithoutWaypoints) {
    EXPECT_FALSE(auto_mode_.can_enter(state_));
}

TEST_F(ModeAutoTest, CanEnterWithWaypoints) {
    Waypoint wp;
    wp.position = NEDPosition(500.0f, 0.0f, -100.0f);
    wp.altitude_msl_m = 100.0f;
    auto_mode_.add_waypoint(wp);
    EXPECT_TRUE(auto_mode_.can_enter(state_));
}

TEST_F(ModeAutoTest, CannotEnterWithoutControllers) {
    ModeAuto no_ctrl;
    Waypoint wp;
    wp.position = NEDPosition(500.0f, 0.0f, -100.0f);
    no_ctrl.add_waypoint(wp);
    EXPECT_FALSE(no_ctrl.can_enter(state_));
}

TEST_F(ModeAutoTest, MissionManagement) {
    EXPECT_EQ(auto_mode_.waypoint_count(), 0);

    Waypoint wp1;
    wp1.position = NEDPosition(500.0f, 0.0f, -100.0f);
    wp1.altitude_msl_m = 100.0f;

    Waypoint wp2;
    wp2.position = NEDPosition(1000.0f, 500.0f, -100.0f);
    wp2.altitude_msl_m = 120.0f;

    EXPECT_TRUE(auto_mode_.add_waypoint(wp1));
    EXPECT_TRUE(auto_mode_.add_waypoint(wp2));
    EXPECT_EQ(auto_mode_.waypoint_count(), 2);

    auto_mode_.clear_mission();
    EXPECT_EQ(auto_mode_.waypoint_count(), 0);
}

TEST_F(ModeAutoTest, SetWaypoints) {
    Waypoint wps[3];
    wps[0].position = NEDPosition(100.0f, 0.0f, -100.0f);
    wps[1].position = NEDPosition(200.0f, 100.0f, -100.0f);
    wps[2].position = NEDPosition(300.0f, 0.0f, -100.0f);

    EXPECT_TRUE(auto_mode_.set_waypoints(wps, 3));
    EXPECT_EQ(auto_mode_.waypoint_count(), 3);

    // Null pointer rejected
    EXPECT_FALSE(auto_mode_.set_waypoints(nullptr, 1));
}

TEST_F(ModeAutoTest, EmptyMissionReturnsError) {
    auto_mode_.enter();
    auto result = auto_mode_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_error());
    EXPECT_EQ(result.error(), ErrorCode::MissionEmpty);
}

TEST_F(ModeAutoTest, NavigatesToWaypoint) {
    Waypoint wp;
    wp.position = NEDPosition(500.0f, 0.0f, -100.0f);
    wp.altitude_msl_m = 100.0f;
    auto_mode_.add_waypoint(wp);
    auto_mode_.enter();

    EXPECT_EQ(auto_mode_.current_waypoint_index(), 0);
    EXPECT_FALSE(auto_mode_.mission_complete());

    auto result = auto_mode_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());

    const auto& s = result.value();
    EXPECT_GE(s.aileron, -1.0f);
    EXPECT_LE(s.aileron, 1.0f);
    EXPECT_GE(s.throttle, 0.0f);
    EXPECT_LE(s.throttle, 1.0f);
}

TEST_F(ModeAutoTest, EnterResetsMission) {
    Waypoint wps[2];
    wps[0].position = NEDPosition(10.0f, 0.0f, -100.0f);
    wps[0].altitude_msl_m = 100.0f;
    wps[0].acceptance_radius_m = 50.0f;
    wps[1].position = NEDPosition(500.0f, 0.0f, -100.0f);
    wps[1].altitude_msl_m = 100.0f;
    auto_mode_.set_waypoints(wps, 2);

    // Move aircraft right on top of first waypoint to trigger advancement
    state_.position = NEDPosition(10.0f, 0.0f, -100.0f);
    auto_mode_.enter();

    // Run a few updates to advance past wp0
    for (int i = 0; i < 5; ++i) {
        auto_mode_.update(state_, rc_, dt_);
    }

    // Re-enter should reset to waypoint 0
    auto_mode_.enter();
    EXPECT_EQ(auto_mode_.current_waypoint_index(), 0);
    EXPECT_FALSE(auto_mode_.mission_complete());
}

// ---------------------------------------------------------------------------
// Integration: full manager with multiple modes
// ---------------------------------------------------------------------------

class FlightModeIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_ = make_level_state();
        rc_ = make_centered_rc();

        attitude_ctrl_ = AttitudeController(AttitudeControllerConfig{});
        attitude_ctrl_.set_sample_rate(100.0f);
        tecs_ctrl_ = TECSController(TECSConfig::Default());
        l1_ctrl_ = L1Controller(L1Config{});

        stabilize_ = ModeStabilize(&attitude_ctrl_);
        fbw_ = ModeFBW(&attitude_ctrl_, &tecs_ctrl_);
        auto_mode_ = ModeAuto(&attitude_ctrl_, &tecs_ctrl_, &l1_ctrl_);

        manager_.register_mode(&manual_);
        manager_.register_mode(&stabilize_);
        manager_.register_mode(&fbw_);
        manager_.register_mode(&auto_mode_);
    }

    FlightModeManager manager_;
    ModeManual manual_;
    ModeStabilize stabilize_;
    ModeFBW fbw_;
    ModeAuto auto_mode_;

    AttitudeController attitude_ctrl_;
    TECSController tecs_ctrl_;
    L1Controller l1_ctrl_;

    AircraftState state_;
    RCInput rc_;
    static constexpr float dt_ = 0.01f;
};

TEST_F(FlightModeIntegrationTest, FullTransitionCycle) {
    // Start in manual
    ASSERT_TRUE(manager_.request_mode(FlightMode::Manual, state_).is_ok());
    auto result = manager_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_ok());

    // Transition to stabilize
    ASSERT_TRUE(manager_.request_mode(FlightMode::Stabilize, state_).is_ok());
    result = manager_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_ok());

    // Transition to FBW
    ASSERT_TRUE(manager_.request_mode(FlightMode::FlyByWire, state_).is_ok());
    result = manager_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_ok());

    // Auto without waypoints should fail
    EXPECT_TRUE(manager_.request_mode(FlightMode::Auto, state_).is_error());

    // Add waypoints and try again
    Waypoint wp;
    wp.position = NEDPosition(500.0f, 0.0f, -100.0f);
    wp.altitude_msl_m = 100.0f;
    auto_mode_.add_waypoint(wp);
    ASSERT_TRUE(manager_.request_mode(FlightMode::Auto, state_).is_ok());
    result = manager_.update(state_, rc_, dt_);
    EXPECT_TRUE(result.is_ok());

    // Back to manual
    ASSERT_TRUE(manager_.request_mode(FlightMode::Manual, state_).is_ok());
    EXPECT_EQ(manager_.current_mode(), FlightMode::Manual);
}

TEST_F(FlightModeIntegrationTest, OutputsAreAlwaysClamped) {
    manager_.request_mode(FlightMode::Manual, state_);

    // Extreme RC inputs
    rc_.channels[0] = 2.0f;   // Over-range
    rc_.channels[1] = -2.0f;
    rc_.channels[2] = 5.0f;
    rc_.channels[3] = -3.0f;

    auto result = manager_.update(state_, rc_, dt_);
    ASSERT_TRUE(result.is_ok());
    const auto& s = result.value();
    EXPECT_GE(s.aileron, -1.0f);
    EXPECT_LE(s.aileron, 1.0f);
    EXPECT_GE(s.elevator, -1.0f);
    EXPECT_LE(s.elevator, 1.0f);
    EXPECT_GE(s.throttle, 0.0f);
    EXPECT_LE(s.throttle, 1.0f);
    EXPECT_GE(s.rudder, -1.0f);
    EXPECT_LE(s.rudder, 1.0f);
}
