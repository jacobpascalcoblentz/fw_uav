#include <gtest/gtest.h>
#include "fw_uav/control/l1_controller.h"

using namespace fw_uav;

class L1ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = L1Controller(config_);
    }

    L1Config config_;
    L1Controller controller_;
};

TEST_F(L1ControllerTest, DefaultConstruction) {
    L1Controller ctrl;
    EXPECT_FALSE(ctrl.last_output().data_valid);
}

TEST_F(L1ControllerTest, ConfigConstruction) {
    L1Config config;
    config.l1_period = 20.0f;
    L1Controller ctrl(config);
    EXPECT_FLOAT_EQ(ctrl.config().l1_period, 20.0f);
}

TEST_F(L1ControllerTest, SetConfig) {
    L1Config new_config;
    new_config.l1_damping = 0.9f;
    controller_.set_config(new_config);
    EXPECT_FLOAT_EQ(controller_.config().l1_damping, 0.9f);
}

TEST_F(L1ControllerTest, L1Distance) {
    float gs = 20.0f; // 20 m/s ground speed
    float l1 = config_.l1_distance(gs);
    // L1 = (1/pi) * damping * period * gs
    float expected = (1.0f / math::PI) * config_.l1_damping * config_.l1_period * gs;
    EXPECT_FLOAT_EQ(l1, expected);
}

TEST_F(L1ControllerTest, PathNavigationLowSpeed) {
    NEDPosition pos{0.0f, 0.0f, 0.0f};
    NEDPosition prev_wp{0.0f, 0.0f, 0.0f};
    NEDPosition next_wp{1000.0f, 0.0f, 0.0f};

    // Below minimum ground speed - should return invalid
    auto output = controller_.navigate_path(pos, 1.0f, 0.0f, prev_wp, next_wp);
    EXPECT_FALSE(output.data_valid);
}

TEST_F(L1ControllerTest, PathNavigationOnTrack) {
    // Aircraft on track, heading toward waypoint
    NEDPosition pos{100.0f, 0.0f, 0.0f};
    NEDPosition prev_wp{0.0f, 0.0f, 0.0f};
    NEDPosition next_wp{1000.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_path(pos, 20.0f, 0.0f, prev_wp, next_wp);
    EXPECT_TRUE(output.data_valid);
    // On track: crosstrack should be ~0
    EXPECT_NEAR(output.crosstrack_error_m, 0.0f, 0.1f);
    // Bearing error should be ~0 (heading matches path)
    EXPECT_NEAR(output.bearing_error_rad, 0.0f, 0.1f);
    // Bank angle should be ~0 (no correction needed)
    EXPECT_NEAR(output.bank_angle_rad, 0.0f, 0.1f);
}

TEST_F(L1ControllerTest, PathNavigationCrosstrack) {
    // Aircraft 50m right of track
    NEDPosition pos{100.0f, 50.0f, 0.0f};
    NEDPosition prev_wp{0.0f, 0.0f, 0.0f};
    NEDPosition next_wp{1000.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_path(pos, 20.0f, 0.0f, prev_wp, next_wp);
    EXPECT_TRUE(output.data_valid);
    // Should have positive crosstrack (right of path)
    EXPECT_GT(output.crosstrack_error_m, 0.0f);
    // Should command a left turn (negative bank) to correct
    EXPECT_LT(output.bank_angle_rad, 0.0f);
}

TEST_F(L1ControllerTest, PathNavigationDegenerate) {
    // Same start and end waypoint - should fall back to direct-to
    NEDPosition pos{100.0f, 50.0f, 0.0f};
    NEDPosition wp{500.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_path(pos, 20.0f, 0.0f, wp, wp);
    EXPECT_TRUE(output.data_valid);
}

TEST_F(L1ControllerTest, WaypointNavigation) {
    NEDPosition pos{0.0f, 0.0f, 0.0f};
    NEDPosition target{1000.0f, 0.0f, 0.0f};

    // Heading north, waypoint is north
    auto output = controller_.navigate_waypoint(pos, 20.0f, 0.0f, target);
    EXPECT_TRUE(output.data_valid);
    EXPECT_NEAR(output.bearing_error_rad, 0.0f, 0.01f);
    EXPECT_NEAR(output.bank_angle_rad, 0.0f, 0.01f);
    EXPECT_NEAR(output.wp_distance_m, 1000.0f, 1.0f);
}

TEST_F(L1ControllerTest, WaypointNavigationTurnNeeded) {
    NEDPosition pos{0.0f, 0.0f, 0.0f};
    NEDPosition target{0.0f, 1000.0f, 0.0f}; // Waypoint is east

    // Heading north, waypoint is east - need right turn
    auto output = controller_.navigate_waypoint(pos, 20.0f, 0.0f, target);
    EXPECT_TRUE(output.data_valid);
    // Should command right bank (positive)
    EXPECT_GT(output.bank_angle_rad, 0.0f);
}

TEST_F(L1ControllerTest, WaypointNavigationLowSpeed) {
    NEDPosition pos{0.0f, 0.0f, 0.0f};
    NEDPosition target{1000.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_waypoint(pos, 1.0f, 0.0f, target);
    EXPECT_FALSE(output.data_valid);
}

TEST_F(L1ControllerTest, WaypointNavigationAtTarget) {
    NEDPosition pos{0.0f, 0.0f, 0.0f};
    NEDPosition target{0.0f, 0.0f, 0.0f}; // On top of waypoint

    auto output = controller_.navigate_waypoint(pos, 20.0f, 0.0f, target);
    EXPECT_TRUE(output.data_valid);
}

TEST_F(L1ControllerTest, LoiterClockwise) {
    // Aircraft east of center, heading north (tangent to CW orbit)
    NEDPosition pos{0.0f, 100.0f, 0.0f};
    NEDPosition center{0.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_loiter(pos, 20.0f, 0.0f, center, 100.0f);
    EXPECT_TRUE(output.data_valid);
    // On the orbit: should have some bank for the turn
}

TEST_F(L1ControllerTest, LoiterCounterClockwise) {
    NEDPosition pos{0.0f, 100.0f, 0.0f};
    NEDPosition center{0.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_loiter(pos, 20.0f, 0.0f, center, -100.0f);
    EXPECT_TRUE(output.data_valid);
}

TEST_F(L1ControllerTest, LoiterLowSpeed) {
    NEDPosition pos{0.0f, 100.0f, 0.0f};
    NEDPosition center{0.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_loiter(pos, 1.0f, 0.0f, center, 60.0f);
    EXPECT_FALSE(output.data_valid);
}

TEST_F(L1ControllerTest, LoiterAtCenter) {
    NEDPosition pos{0.0f, 0.0f, 0.0f}; // On top of center
    NEDPosition center{0.0f, 0.0f, 0.0f};

    // Should handle gracefully without div by zero
    auto output = controller_.navigate_loiter(pos, 20.0f, 0.0f, center, 60.0f);
    EXPECT_TRUE(output.data_valid);
}

TEST_F(L1ControllerTest, BankAngleLimiting) {
    // Extreme situation - big crosstrack error
    NEDPosition pos{0.0f, 500.0f, 0.0f};
    NEDPosition prev_wp{0.0f, 0.0f, 0.0f};
    NEDPosition next_wp{1000.0f, 0.0f, 0.0f};

    auto output = controller_.navigate_path(pos, 20.0f, 0.0f, prev_wp, next_wp);
    EXPECT_TRUE(output.data_valid);
    // Bank should be limited
    EXPECT_GE(output.bank_angle_rad, -config_.bank_limit_rad);
    EXPECT_LE(output.bank_angle_rad, config_.bank_limit_rad);
}

TEST_F(L1ControllerTest, HasReachedWaypoint) {
    NEDPosition prev_wp{0.0f, 0.0f, 0.0f};
    NEDPosition next_wp{1000.0f, 0.0f, 0.0f};

    // Before waypoint
    NEDPosition before{900.0f, 0.0f, 0.0f};
    EXPECT_FALSE(controller_.has_reached_waypoint(before, prev_wp, next_wp));

    // Past waypoint
    NEDPosition past{1100.0f, 0.0f, 0.0f};
    EXPECT_TRUE(controller_.has_reached_waypoint(past, prev_wp, next_wp));
}

TEST_F(L1ControllerTest, Reset) {
    NEDPosition pos{0.0f, 0.0f, 0.0f};
    NEDPosition target{1000.0f, 0.0f, 0.0f};

    controller_.navigate_waypoint(pos, 20.0f, 0.0f, target);
    EXPECT_TRUE(controller_.last_output().data_valid);

    controller_.reset();
    EXPECT_FALSE(controller_.last_output().data_valid);
}
