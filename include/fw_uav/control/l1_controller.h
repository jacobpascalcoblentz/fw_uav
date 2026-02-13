#pragma once

#include <cmath>
#include <fw_uav/core/types.h>

namespace fw_uav {

// Configuration for the L1 navigation controller
struct L1Config {
    float l1_period{17.0f};       // L1 tracking period in seconds (larger = smoother)
    float l1_damping{0.75f};      // Damping ratio (0.707 = critically damped)
    float bank_limit_rad{0.7854f}; // Max bank angle (default 45 deg)
    float loiter_radius_m{60.0f};  // Default loiter radius

    // Compute the L1 distance from period and groundspeed
    float l1_distance(float ground_speed_mps) const {
        // L1 = (1 / pi) * damping * period * ground_speed
        return (1.0f / math::PI) * l1_damping * l1_period * ground_speed_mps;
    }
};

// Output from the L1 controller
struct L1Output {
    float lateral_accel_mps2{0.0f}; // Lateral acceleration demand (m/s^2)
    float bank_angle_rad{0.0f};     // Commanded bank angle (rad)
    float bearing_error_rad{0.0f};  // Bearing error to target (rad)
    float crosstrack_error_m{0.0f}; // Crosstrack distance from path (m, positive right)
    float wp_distance_m{0.0f};      // Distance to active waypoint (m)
    bool data_valid{false};          // True if output is valid
};

// L1 navigation controller for lateral/directional guidance
//
// Implements the L1 guidance law for fixed-wing aircraft, computing roll
// commands to track waypoints and paths. Based on:
//   S. Park, J. Deyst, J.P. How, "A New Nonlinear Guidance Logic for
//   Trajectory Tracking," AIAA GNC 2004 / 2007.
//
// The controller supports two primary modes:
//   1. Path following: Track a line between two waypoints
//   2. Loiter: Orbit around a point at a specified radius
class L1Controller {
public:
    L1Controller() = default;
    explicit L1Controller(const L1Config& config) : config_(config) {}

    void set_config(const L1Config& config) { config_ = config; }
    const L1Config& config() const { return config_; }

    // Navigate along a straight line from prev_wp to next_wp
    //
    // @param position_ned  Current aircraft position in NED (m)
    // @param ground_speed  Current ground speed (m/s)
    // @param course_rad    Current ground track heading (rad, NED)
    // @param prev_wp       Start of the path segment in NED (m)
    // @param next_wp       End of the path segment in NED (m)
    // @return L1Output with bank angle command and diagnostics
    L1Output navigate_path(const NEDPosition& position_ned,
                           float ground_speed,
                           float course_rad,
                           const NEDPosition& prev_wp,
                           const NEDPosition& next_wp);

    // Navigate to a single waypoint (direct-to)
    //
    // @param position_ned  Current aircraft position in NED (m)
    // @param ground_speed  Current ground speed (m/s)
    // @param course_rad    Current ground track heading (rad, NED)
    // @param target_wp     Target waypoint in NED (m)
    // @return L1Output with bank angle command
    L1Output navigate_waypoint(const NEDPosition& position_ned,
                               float ground_speed,
                               float course_rad,
                               const NEDPosition& target_wp);

    // Loiter around a center point at a specified radius
    //
    // @param position_ned  Current aircraft position in NED (m)
    // @param ground_speed  Current ground speed (m/s)
    // @param course_rad    Current ground track heading (rad, NED)
    // @param center        Center of the loiter circle in NED (m)
    // @param radius_m      Loiter radius (positive = clockwise, negative = CCW)
    // @return L1Output with bank angle command
    L1Output navigate_loiter(const NEDPosition& position_ned,
                             float ground_speed,
                             float course_rad,
                             const NEDPosition& center,
                             float radius_m);

    // Has the aircraft passed the waypoint (for sequencing)?
    // Returns true when the aircraft has passed the perpendicular line through
    // next_wp along the path from prev_wp to next_wp.
    bool has_reached_waypoint(const NEDPosition& position_ned,
                              const NEDPosition& prev_wp,
                              const NEDPosition& next_wp) const;

    // Reset controller state
    void reset();

    // Get the last computed output
    const L1Output& last_output() const { return output_; }

private:
    // Compute bank angle from lateral acceleration demand
    float lateral_accel_to_bank(float lateral_accel, float ground_speed) const;

    // Minimum ground speed for valid guidance (m/s)
    static constexpr float MIN_GROUND_SPEED = 3.0f;

    // Gravity constant
    static constexpr float GRAVITY = 9.80665f;

    L1Config config_;
    L1Output output_;
};

} // namespace fw_uav
