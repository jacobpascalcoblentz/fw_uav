#pragma once

#include "fw_uav/control/pid.h"
#include "fw_uav/core/types.h"
#include "fw_uav/core/result.h"

namespace fw_uav {

// Control mode for the attitude controller
enum class AttitudeControlMode : uint8_t {
    Rate,       // Direct rate control (acro/rate mode)
    Angle,      // Angle stabilization (stabilize mode)
};

// Configuration for the attitude controller
struct AttitudeControllerConfig {
    // Roll axis - cascaded: angle P -> rate PID
    PIDConfig roll_angle{PIDConfig::P(4.0f, -1.0f, 1.0f)};
    PIDConfig roll_rate{PIDConfig::PID(0.1f, 0.02f, 0.01f, -1.0f, 1.0f)};

    // Pitch axis - cascaded: angle P -> rate PID
    PIDConfig pitch_angle{PIDConfig::P(3.0f, -1.0f, 1.0f)};
    PIDConfig pitch_rate{PIDConfig::PID(0.08f, 0.015f, 0.008f, -1.0f, 1.0f)};

    // Yaw axis - cascaded: angle P -> rate PID
    PIDConfig yaw_angle{PIDConfig::P(2.0f, -1.0f, 1.0f)};
    PIDConfig yaw_rate{PIDConfig::PID(0.05f, 0.01f, 0.0f, -1.0f, 1.0f)};

    // Angle limits (radians)
    float max_roll_rad{math::PI / 4.0f};      // 45 degrees
    float max_pitch_rad{math::PI / 6.0f};     // 30 degrees

    // Rate limits (rad/s) - applied in both rate and angle modes
    float max_roll_rate_radps{math::PI};       // 180 deg/s
    float max_pitch_rate_radps{math::PI * 0.5f}; // 90 deg/s
    float max_yaw_rate_radps{math::PI * 0.5f};   // 90 deg/s
};

// Setpoint for the attitude controller
struct AttitudeSetpoint {
    float roll{0.0f};   // Roll angle (rad) in Angle mode, roll rate (rad/s) in Rate mode
    float pitch{0.0f};  // Pitch angle (rad) in Angle mode, pitch rate (rad/s) in Rate mode
    float yaw{0.0f};    // Yaw rate (rad/s) in both modes (yaw angle hold handled separately)
};

// Output from the attitude controller (normalized surface commands)
struct AttitudeControlOutput {
    float aileron{0.0f};    // -1.0 to +1.0
    float elevator{0.0f};   // -1.0 to +1.0
    float rudder{0.0f};     // -1.0 to +1.0
};

// Cascaded attitude controller for fixed-wing aircraft
//
// In Angle mode: setpoint angles -> angle PID -> rate setpoint -> rate PID -> surface output
// In Rate mode: setpoint rates -> rate PID -> surface output (angle PIDs bypassed)
class AttitudeController {
public:
    AttitudeController() = default;
    explicit AttitudeController(const AttitudeControllerConfig& config);

    // Set configuration (can be called at runtime for gain tuning)
    void set_config(const AttitudeControllerConfig& config);
    const AttitudeControllerConfig& config() const { return config_; }

    // Set the control mode
    void set_mode(AttitudeControlMode mode);
    AttitudeControlMode mode() const { return mode_; }

    // Main update - computes surface outputs from setpoint and current state
    // @param setpoint: Desired attitude (angles in Angle mode, rates in Rate mode)
    // @param euler: Current Euler angles
    // @param body_rates: Current body-frame angular rates (rad/s) [p, q, r]
    // @param dt: Time step in seconds
    Result<AttitudeControlOutput> update(
        const AttitudeSetpoint& setpoint,
        const EulerAngles& euler,
        const Vec3f& body_rates,
        float dt);

    // Reset all controller state (call on mode transitions)
    void reset();

    // Set the sample rate for derivative filters
    void set_sample_rate(float hz);

private:
    void apply_config();

    AttitudeControllerConfig config_;
    AttitudeControlMode mode_{AttitudeControlMode::Angle};

    // Roll axis
    CascadedPID roll_controller_;

    // Pitch axis
    CascadedPID pitch_controller_;

    // Yaw axis
    CascadedPID yaw_controller_;

    bool initialized_{false};
};

} // namespace fw_uav
