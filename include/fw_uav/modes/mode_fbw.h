#pragma once

#include "fw_uav/modes/flight_mode_manager.h"
#include "fw_uav/control/attitude_controller.h"
#include "fw_uav/control/tecs.h"

namespace fw_uav {

// Configuration for fly-by-wire mode
struct FBWModeConfig {
    // Maximum attitude commands from RC sticks (radians)
    float max_roll_cmd_rad{math::PI / 4.0f};         // 45 degrees
    float max_yaw_rate_cmd_radps{math::PI / 4.0f};   // 45 deg/s

    // Airspeed and altitude targets
    float cruise_airspeed_mps{18.0f};
    float target_altitude_m{100.0f};     // Initial altitude setpoint (MSL)

    // RC rate limits for altitude and airspeed adjustment
    float altitude_rate_mps{2.0f};       // Max altitude change rate from stick
    float airspeed_rate_mps2{1.0f};      // Max airspeed change rate from stick

    // Airspeed limits
    float min_airspeed_mps{12.0f};
    float max_airspeed_mps{30.0f};
};

// Fly-By-Wire mode: rate-limited attitude from RC with TECS
//
// RC roll stick commands bank angle (same as stabilize). RC pitch stick
// adjusts altitude setpoint up/down. TECS coordinates throttle and pitch
// to hold both airspeed and altitude. This mode provides envelope
// protection: the aircraft cannot stall or overspeed.
class ModeFBW : public FlightModeBase {
public:
    ModeFBW() = default;

    explicit ModeFBW(AttitudeController* attitude_ctrl,
                     TECSController* tecs_ctrl,
                     const FBWModeConfig& config = {})
        : attitude_ctrl_(attitude_ctrl), tecs_ctrl_(tecs_ctrl), config_(config) {}

    void set_attitude_controller(AttitudeController* ctrl) { attitude_ctrl_ = ctrl; }
    void set_tecs_controller(TECSController* ctrl) { tecs_ctrl_ = ctrl; }
    void set_config(const FBWModeConfig& config) { config_ = config; }
    const FBWModeConfig& fbw_config() const { return config_; }

    FlightMode id() const override { return FlightMode::FlyByWire; }
    const char* name() const override { return "FlyByWire"; }

    bool can_enter(const AircraftState& state) const override {
        // Need both controllers and a valid airspeed reading
        return attitude_ctrl_ != nullptr
            && tecs_ctrl_ != nullptr
            && state.airspeed_mps > 0.0f;
    }

    void enter() override {
        if (attitude_ctrl_) {
            attitude_ctrl_->set_mode(AttitudeControlMode::Angle);
            attitude_ctrl_->reset();
        }
        if (tecs_ctrl_) {
            tecs_ctrl_->reset();
        }
        altitude_setpoint_m_ = 0.0f;  // Will be initialized on first update
        airspeed_setpoint_mps_ = config_.cruise_airspeed_mps;
        first_update_ = true;
    }

    void exit() override {
        first_update_ = true;
    }

    Result<ControlSurfaces> update(const AircraftState& state,
                                   const RCInput& rc,
                                   float dt) override {
        if (!attitude_ctrl_ || !tecs_ctrl_) {
            return ErrorCode::ControlNotInitialized;
        }
        if (!rc.is_valid()) {
            return ErrorCode::RCLost;
        }

        // Initialize altitude setpoint to current altitude on first update
        if (first_update_) {
            altitude_setpoint_m_ = state.altitude_msl_m;
            first_update_ = false;
        }

        // RC pitch stick adjusts altitude setpoint
        altitude_setpoint_m_ += rc.pitch() * config_.altitude_rate_mps * dt;

        // RC throttle adjusts airspeed setpoint (mapped from 0..1 to min..max)
        airspeed_setpoint_mps_ = math::lerp(config_.min_airspeed_mps,
                                             config_.max_airspeed_mps,
                                             rc.throttle());

        // Run TECS for throttle and pitch reference
        TECSOutput tecs_out = tecs_ctrl_->update(
            airspeed_setpoint_mps_, altitude_setpoint_m_,
            state.airspeed_mps, state.altitude_msl_m,
            state.climb_rate_mps(), dt);

        // RC roll stick commands bank angle, TECS provides pitch
        AttitudeSetpoint setpoint;
        setpoint.roll = rc.roll() * config_.max_roll_cmd_rad;
        setpoint.pitch = tecs_out.pitch_ref_rad;
        setpoint.yaw = rc.yaw() * config_.max_yaw_rate_cmd_radps;

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
        surfaces.throttle = tecs_out.throttle;  // TECS controls throttle
        surfaces.flaps = 0.0f;
        surfaces.clamp();

        return surfaces;
    }

    // Get current setpoints for telemetry
    float altitude_setpoint() const { return altitude_setpoint_m_; }
    float airspeed_setpoint() const { return airspeed_setpoint_mps_; }

private:
    AttitudeController* attitude_ctrl_{nullptr};
    TECSController* tecs_ctrl_{nullptr};
    FBWModeConfig config_;

    float altitude_setpoint_m_{0.0f};
    float airspeed_setpoint_mps_{18.0f};
    bool first_update_{true};
};

} // namespace fw_uav
