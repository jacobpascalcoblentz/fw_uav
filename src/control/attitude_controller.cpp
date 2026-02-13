#include "fw_uav/control/attitude_controller.h"

namespace fw_uav {

AttitudeController::AttitudeController(const AttitudeControllerConfig& config)
    : config_(config) {
    apply_config();
}

void AttitudeController::set_config(const AttitudeControllerConfig& config) {
    config_ = config;
    apply_config();
}

void AttitudeController::set_mode(AttitudeControlMode mode) {
    if (mode != mode_) {
        mode_ = mode;
        reset();
    }
}

Result<AttitudeControlOutput> AttitudeController::update(
    const AttitudeSetpoint& setpoint,
    const EulerAngles& euler,
    const Vec3f& body_rates,
    float dt) {

    if (dt <= 0.0f) {
        return ErrorCode::InvalidParameter;
    }

    AttitudeControlOutput output;

    if (mode_ == AttitudeControlMode::Angle) {
        // Angle mode: setpoint is desired attitude angles
        // Clamp setpoints to configured limits
        float roll_cmd = math::constrain(setpoint.roll,
            -config_.max_roll_rad, config_.max_roll_rad);
        float pitch_cmd = math::constrain(setpoint.pitch,
            -config_.max_pitch_rad, config_.max_pitch_rad);

        // Roll: cascaded angle -> rate -> aileron
        output.aileron = roll_controller_.update(
            roll_cmd, euler.roll, body_rates.x, dt);

        // Pitch: cascaded angle -> rate -> elevator
        output.elevator = pitch_controller_.update(
            pitch_cmd, euler.pitch, body_rates.y, dt);

        // Yaw: setpoint is always a rate command, use cascaded with 0 angle error
        // to get rate damping, plus the commanded rate
        output.rudder = yaw_controller_.inner().update(
            setpoint.yaw, body_rates.z, dt);

    } else {
        // Rate mode: setpoint is desired body rates directly
        // Clamp rate setpoints
        float roll_rate_cmd = math::constrain(setpoint.roll,
            -config_.max_roll_rate_radps, config_.max_roll_rate_radps);
        float pitch_rate_cmd = math::constrain(setpoint.pitch,
            -config_.max_pitch_rate_radps, config_.max_pitch_rate_radps);
        float yaw_rate_cmd = math::constrain(setpoint.yaw,
            -config_.max_yaw_rate_radps, config_.max_yaw_rate_radps);

        // Rate-only: bypass outer (angle) loop, use inner (rate) loop directly
        output.aileron = roll_controller_.inner().update(
            roll_rate_cmd, body_rates.x, dt);
        output.elevator = pitch_controller_.inner().update(
            pitch_rate_cmd, body_rates.y, dt);
        output.rudder = yaw_controller_.inner().update(
            yaw_rate_cmd, body_rates.z, dt);
    }

    initialized_ = true;
    return output;
}

void AttitudeController::reset() {
    roll_controller_.reset();
    pitch_controller_.reset();
    yaw_controller_.reset();
    initialized_ = false;
}

void AttitudeController::set_sample_rate(float hz) {
    roll_controller_.set_sample_rate(hz);
    pitch_controller_.set_sample_rate(hz);
    yaw_controller_.set_sample_rate(hz);
}

void AttitudeController::apply_config() {
    roll_controller_.set_outer_config(config_.roll_angle);
    roll_controller_.set_inner_config(config_.roll_rate);
    pitch_controller_.set_outer_config(config_.pitch_angle);
    pitch_controller_.set_inner_config(config_.pitch_rate);
    yaw_controller_.set_outer_config(config_.yaw_angle);
    yaw_controller_.set_inner_config(config_.yaw_rate);
}

} // namespace fw_uav
