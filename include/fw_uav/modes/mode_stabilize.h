#pragma once

#include "fw_uav/modes/flight_mode_manager.h"
#include "fw_uav/control/attitude_controller.h"

namespace fw_uav {

// Configuration for stabilize mode
struct StabilizeModeConfig {
    // Maximum attitude commands from RC sticks (radians)
    float max_roll_cmd_rad{math::PI / 4.0f};     // 45 degrees
    float max_pitch_cmd_rad{math::PI / 6.0f};     // 30 degrees
    float max_yaw_rate_cmd_radps{math::PI / 4.0f}; // 45 deg/s
};

// Stabilize mode: attitude hold with manual throttle
//
// RC sticks command desired roll/pitch angles. The attitude controller
// maintains those angles using cascaded PID loops. Throttle passes
// through directly from the RC transmitter. This provides basic
// self-leveling while the pilot retains throttle authority.
class ModeStabilize : public FlightModeBase {
public:
    ModeStabilize() = default;

    explicit ModeStabilize(AttitudeController* attitude_ctrl,
                           const StabilizeModeConfig& config = {})
        : attitude_ctrl_(attitude_ctrl), config_(config) {}

    void set_attitude_controller(AttitudeController* ctrl) { attitude_ctrl_ = ctrl; }
    void set_config(const StabilizeModeConfig& config) { config_ = config; }

    FlightMode id() const override { return FlightMode::Stabilize; }
    const char* name() const override { return "Stabilize"; }

    bool can_enter(const AircraftState& /*state*/) const override {
        return attitude_ctrl_ != nullptr;
    }

    void enter() override {
        if (attitude_ctrl_) {
            attitude_ctrl_->set_mode(AttitudeControlMode::Angle);
            attitude_ctrl_->reset();
        }
    }

    void exit() override {
        // Nothing to clean up
    }

    Result<ControlSurfaces> update(const AircraftState& state,
                                   const RCInput& rc,
                                   float dt) override {
        if (!attitude_ctrl_) {
            return ErrorCode::ControlNotInitialized;
        }
        if (!rc.is_valid()) {
            return ErrorCode::RCLost;
        }

        // Map RC sticks to attitude setpoints
        AttitudeSetpoint setpoint;
        setpoint.roll = rc.roll() * config_.max_roll_cmd_rad;
        setpoint.pitch = rc.pitch() * config_.max_pitch_cmd_rad;
        setpoint.yaw = rc.yaw() * config_.max_yaw_rate_cmd_radps;

        // Run attitude controller
        auto result = attitude_ctrl_->update(
            setpoint, state.euler, state.angular_rate_radps, dt);

        if (result.is_error()) {
            return result.error();
        }

        const auto& att_out = result.value();

        ControlSurfaces surfaces;
        surfaces.aileron = att_out.aileron;
        surfaces.elevator = att_out.elevator;
        surfaces.rudder = att_out.rudder;
        surfaces.throttle = rc.throttle();  // Manual throttle
        surfaces.flaps = 0.0f;
        surfaces.clamp();

        return surfaces;
    }

private:
    AttitudeController* attitude_ctrl_{nullptr};
    StabilizeModeConfig config_;
};

} // namespace fw_uav
