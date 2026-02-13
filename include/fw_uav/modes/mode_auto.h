#pragma once

#include "fw_uav/modes/flight_mode_manager.h"
#include "fw_uav/control/attitude_controller.h"
#include "fw_uav/control/tecs.h"
#include "fw_uav/control/l1_controller.h"

namespace fw_uav {

// A single waypoint in the mission
struct Waypoint {
    NEDPosition position{};         // NED relative to home
    float altitude_msl_m{100.0f};   // Target altitude (MSL)
    float airspeed_mps{0.0f};       // Target airspeed (0 = use default)
    float acceptance_radius_m{30.0f}; // Waypoint reached within this radius
};

// Configuration for auto mode
struct AutoModeConfig {
    static constexpr int MAX_WAYPOINTS = 64;

    float default_airspeed_mps{18.0f};
    float default_altitude_m{100.0f};
    float max_yaw_rate_cmd_radps{math::PI / 4.0f};

    // Waypoint acceptance
    float default_acceptance_radius_m{30.0f};
};

// Auto mode: waypoint following using L1 + TECS + AttitudeController
//
// The L1 controller provides lateral guidance (bank angle commands) to
// track paths between waypoints. TECS handles longitudinal control
// (throttle + pitch) to maintain airspeed and altitude. The attitude
// controller stabilizes the aircraft to follow these commands.
class ModeAuto : public FlightModeBase {
public:
    ModeAuto() = default;

    explicit ModeAuto(AttitudeController* attitude_ctrl,
                      TECSController* tecs_ctrl,
                      L1Controller* l1_ctrl,
                      const AutoModeConfig& config = {})
        : attitude_ctrl_(attitude_ctrl)
        , tecs_ctrl_(tecs_ctrl)
        , l1_ctrl_(l1_ctrl)
        , config_(config) {}

    void set_attitude_controller(AttitudeController* ctrl) { attitude_ctrl_ = ctrl; }
    void set_tecs_controller(TECSController* ctrl) { tecs_ctrl_ = ctrl; }
    void set_l1_controller(L1Controller* ctrl) { l1_ctrl_ = ctrl; }
    void set_config(const AutoModeConfig& config) { config_ = config; }

    FlightMode id() const override { return FlightMode::Auto; }
    const char* name() const override { return "Auto"; }

    bool can_enter(const AircraftState& state) const override {
        return attitude_ctrl_ != nullptr
            && tecs_ctrl_ != nullptr
            && l1_ctrl_ != nullptr
            && state.airspeed_mps > 0.0f
            && waypoint_count_ > 0;
    }

    void enter() override {
        if (attitude_ctrl_) {
            attitude_ctrl_->set_mode(AttitudeControlMode::Angle);
            attitude_ctrl_->reset();
        }
        if (tecs_ctrl_) {
            tecs_ctrl_->reset();
        }
        if (l1_ctrl_) {
            l1_ctrl_->reset();
        }
        // Start from the beginning of the mission
        current_wp_index_ = 0;
        mission_complete_ = false;
    }

    void exit() override {
        // Nothing to clean up
    }

    // Mission management
    bool set_waypoints(const Waypoint* waypoints, int count) {
        if (count < 0 || count > AutoModeConfig::MAX_WAYPOINTS || waypoints == nullptr) {
            return false;
        }
        for (int i = 0; i < count; ++i) {
            waypoints_[i] = waypoints[i];
        }
        waypoint_count_ = count;
        current_wp_index_ = 0;
        mission_complete_ = false;
        return true;
    }

    bool add_waypoint(const Waypoint& wp) {
        if (waypoint_count_ >= AutoModeConfig::MAX_WAYPOINTS) {
            return false;
        }
        waypoints_[waypoint_count_++] = wp;
        return true;
    }

    void clear_mission() {
        waypoint_count_ = 0;
        current_wp_index_ = 0;
        mission_complete_ = false;
    }

    int waypoint_count() const { return waypoint_count_; }
    int current_waypoint_index() const { return current_wp_index_; }
    bool mission_complete() const { return mission_complete_; }

    Result<ControlSurfaces> update(const AircraftState& state,
                                   const RCInput& /*rc*/,
                                   float dt) override {
        if (!attitude_ctrl_ || !tecs_ctrl_ || !l1_ctrl_) {
            return ErrorCode::ControlNotInitialized;
        }
        if (waypoint_count_ == 0) {
            return ErrorCode::MissionEmpty;
        }

        // If mission is complete, loiter at the last waypoint
        if (mission_complete_) {
            return loiter_at_waypoint(state, waypoints_[waypoint_count_ - 1], dt);
        }

        const Waypoint& target = waypoints_[current_wp_index_];

        // Check if we've reached the current waypoint
        if (current_wp_index_ > 0) {
            const Waypoint& prev = waypoints_[current_wp_index_ - 1];
            if (l1_ctrl_->has_reached_waypoint(state.position, prev.position, target.position)) {
                advance_waypoint();
                if (mission_complete_) {
                    return loiter_at_waypoint(state, waypoints_[waypoint_count_ - 1], dt);
                }
            }
        } else {
            // For the first waypoint, check distance directly
            Vec3f diff = target.position.to_vec3f() - state.position.to_vec3f();
            float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            if (dist < target.acceptance_radius_m) {
                advance_waypoint();
                if (mission_complete_) {
                    return loiter_at_waypoint(state, waypoints_[waypoint_count_ - 1], dt);
                }
            }
        }

        // Navigate path segment
        return navigate_to_waypoint(state, dt);
    }

private:
    void advance_waypoint() {
        current_wp_index_++;
        if (current_wp_index_ >= waypoint_count_) {
            mission_complete_ = true;
            current_wp_index_ = waypoint_count_ - 1;
        }
    }

    Result<ControlSurfaces> navigate_to_waypoint(const AircraftState& state, float dt) {
        const Waypoint& target = waypoints_[current_wp_index_];

        // L1 lateral guidance
        L1Output l1_out;
        if (current_wp_index_ > 0) {
            const Waypoint& prev = waypoints_[current_wp_index_ - 1];
            l1_out = l1_ctrl_->navigate_path(
                state.position, state.ground_speed_mps(),
                state.course_rad(), prev.position, target.position);
        } else {
            l1_out = l1_ctrl_->navigate_waypoint(
                state.position, state.ground_speed_mps(),
                state.course_rad(), target.position);
        }

        // TECS longitudinal guidance
        float airspeed_sp = target.airspeed_mps > 0.0f
                            ? target.airspeed_mps
                            : config_.default_airspeed_mps;

        TECSOutput tecs_out = tecs_ctrl_->update(
            airspeed_sp, target.altitude_msl_m,
            state.airspeed_mps, state.altitude_msl_m,
            state.climb_rate_mps(), dt);

        // Attitude controller: bank from L1, pitch from TECS
        AttitudeSetpoint setpoint;
        setpoint.roll = l1_out.bank_angle_rad;
        setpoint.pitch = tecs_out.pitch_ref_rad;
        setpoint.yaw = 0.0f;  // Coordinated turn handled by roll

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
        surfaces.throttle = tecs_out.throttle;
        surfaces.flaps = 0.0f;
        surfaces.clamp();

        return surfaces;
    }

    Result<ControlSurfaces> loiter_at_waypoint(const AircraftState& state,
                                                const Waypoint& wp,
                                                float dt) {
        // Loiter around the last waypoint
        L1Output l1_out = l1_ctrl_->navigate_loiter(
            state.position, state.ground_speed_mps(),
            state.course_rad(), wp.position,
            l1_ctrl_->config().loiter_radius_m);

        float airspeed_sp = wp.airspeed_mps > 0.0f
                            ? wp.airspeed_mps
                            : config_.default_airspeed_mps;

        TECSOutput tecs_out = tecs_ctrl_->update(
            airspeed_sp, wp.altitude_msl_m,
            state.airspeed_mps, state.altitude_msl_m,
            state.climb_rate_mps(), dt);

        AttitudeSetpoint setpoint;
        setpoint.roll = l1_out.bank_angle_rad;
        setpoint.pitch = tecs_out.pitch_ref_rad;
        setpoint.yaw = 0.0f;

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
        surfaces.throttle = tecs_out.throttle;
        surfaces.flaps = 0.0f;
        surfaces.clamp();

        return surfaces;
    }

    AttitudeController* attitude_ctrl_{nullptr};
    TECSController* tecs_ctrl_{nullptr};
    L1Controller* l1_ctrl_{nullptr};
    AutoModeConfig config_;

    Waypoint waypoints_[AutoModeConfig::MAX_WAYPOINTS]{};
    int waypoint_count_{0};
    int current_wp_index_{0};
    bool mission_complete_{false};
};

} // namespace fw_uav
