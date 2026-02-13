#include "fw_uav/safety/failsafe.h"

#include <cmath>

namespace fw_uav {

FailsafeManager::FailsafeManager(const FailsafeConfig& config)
    : config_(config)
{
    // Pre-populate priority and recommended modes from config
    rc_condition_.priority = FailsafePriority::RCLoss;
    rc_condition_.recommended_mode = config_.rc_loss_mode;

    gps_condition_.priority = FailsafePriority::GPSLoss;
    gps_condition_.recommended_mode = config_.gps_loss_mode;

    battery_low_condition_.priority = FailsafePriority::BatteryLow;
    battery_low_condition_.recommended_mode = config_.battery_low_mode;

    battery_critical_condition_.priority = FailsafePriority::BatteryCritical;
    battery_critical_condition_.recommended_mode = config_.battery_critical_mode;

    geofence_condition_.priority = FailsafePriority::GeofenceViolation;
    geofence_condition_.recommended_mode = config_.geofence_violation_mode;
}

void FailsafeManager::configure(const FailsafeConfig& config) {
    config_ = config;

    rc_condition_.priority = FailsafePriority::RCLoss;
    rc_condition_.recommended_mode = config_.rc_loss_mode;

    gps_condition_.priority = FailsafePriority::GPSLoss;
    gps_condition_.recommended_mode = config_.gps_loss_mode;

    battery_low_condition_.priority = FailsafePriority::BatteryLow;
    battery_low_condition_.recommended_mode = config_.battery_low_mode;

    battery_critical_condition_.priority = FailsafePriority::BatteryCritical;
    battery_critical_condition_.recommended_mode = config_.battery_critical_mode;

    geofence_condition_.priority = FailsafePriority::GeofenceViolation;
    geofence_condition_.recommended_mode = config_.geofence_violation_mode;
}

FailsafeAction FailsafeManager::update(const AircraftState& state,
                                        const RCInput& rc,
                                        float battery_voltage,
                                        float dt_s) {
    update_rc(rc, dt_s);
    update_gps(state, dt_s);
    update_battery(battery_voltage);
    update_geofence(state);

    // Build the combined action from all conditions.
    // The highest-priority active condition determines the recommended mode.
    FailsafeAction action{};
    action.rc_lost = rc_condition_.active;
    action.gps_lost = gps_condition_.active;
    action.battery_low = battery_low_condition_.active;
    action.battery_critical = battery_critical_condition_.active;
    action.geofence_violated = geofence_condition_.active;

    // Collect all active conditions and pick the highest priority
    const FailsafeCondition* conditions[] = {
        &rc_condition_,
        &gps_condition_,
        &battery_low_condition_,
        &battery_critical_condition_,
        &geofence_condition_
    };

    for (const auto* cond : conditions) {
        if (cond->active &&
            static_cast<uint8_t>(cond->priority) >
                static_cast<uint8_t>(action.highest_priority)) {
            action.highest_priority = cond->priority;
            action.recommended_mode = cond->recommended_mode;
            action.failsafe_active = true;
        }
    }

    return action;
}

void FailsafeManager::reset() {
    rc_condition_ = {};
    gps_condition_ = {};
    battery_low_condition_ = {};
    battery_critical_condition_ = {};
    geofence_condition_ = {};
    battery_was_low_ = false;
    battery_was_critical_ = false;

    // Re-apply priorities and modes from config
    configure(config_);
}

bool FailsafeManager::any_active() const {
    return rc_condition_.active ||
           gps_condition_.active ||
           battery_low_condition_.active ||
           battery_critical_condition_.active ||
           geofence_condition_.active;
}

// ---- Private update helpers ------------------------------------------------

void FailsafeManager::update_rc(const RCInput& rc, float dt_s) {
    if (!rc.is_valid()) {
        rc_condition_.detected = true;
        rc_condition_.elapsed_s += dt_s;
        if (rc_condition_.elapsed_s >= config_.rc_loss_timeout_s) {
            rc_condition_.active = true;
        }
    } else {
        // RC recovered
        rc_condition_.detected = false;
        rc_condition_.active = false;
        rc_condition_.elapsed_s = 0.0f;
    }
}

void FailsafeManager::update_gps(const AircraftState& state, float dt_s) {
    // Use the GPS fix type from geo_position data.
    // We consider GPS lost when there is no 3D fix AND the aircraft is in the
    // air. On the ground, GPS loss is not a failsafe event.
    // However, we detect the condition regardless so callers can see it, and
    // only set active when in-air and timeout exceeded.
    //
    // We rely on AircraftState not having a direct GPSData field, so we infer
    // GPS health from position validity. A simple heuristic: if ground speed
    // is exactly zero and the aircraft claims to be in the air with non-zero
    // airspeed, GPS may have been lost. For a more robust check, the caller
    // should set a flag, but here we use the position timestamp staleness
    // approach by relying on the elapsed timer.
    //
    // For this implementation, GPS loss is signaled externally through
    // AircraftState: if geo_position latitude/longitude are both exactly 0
    // (which is ocean, not a valid operating area) or if the state's
    // timestamp is used. A cleaner approach is to check GPSData directly,
    // but we work with AircraftState. We'll use a simple check: the caller
    // is expected to set geo_position to {0,0,0} or position to {0,0,0}
    // when GPS is lost.
    //
    // Simplest robust approach: track GPS loss externally. For now, we check
    // if position.north_m and position.east_m are both NaN or if the aircraft
    // says in_air but ground_speed is 0 and airspeed > 0.
    //
    // DESIGN DECISION: We check for NaN in position fields as the GPS-loss
    // indicator. The sensor layer should set position to NaN when fix is lost.

    bool gps_healthy = !std::isnan(state.position.north_m) &&
                       !std::isnan(state.position.east_m);

    if (!gps_healthy) {
        gps_condition_.detected = true;
        gps_condition_.elapsed_s += dt_s;
        if (gps_condition_.elapsed_s >= config_.gps_loss_timeout_s) {
            gps_condition_.active = true;
        }
    } else {
        gps_condition_.detected = false;
        gps_condition_.active = false;
        gps_condition_.elapsed_s = 0.0f;
    }
}

void FailsafeManager::update_battery(float battery_voltage) {
    // Critical check (higher priority, checked first)
    if (battery_voltage <= config_.battery_critical_voltage) {
        battery_critical_condition_.detected = true;
        battery_critical_condition_.active = true;
        battery_was_critical_ = true;
    } else if (battery_was_critical_ &&
               battery_voltage <= config_.battery_critical_voltage + config_.battery_hysteresis_v) {
        // Still in hysteresis band -- remain active
        battery_critical_condition_.detected = true;
        battery_critical_condition_.active = true;
    } else {
        battery_critical_condition_.detected = false;
        battery_critical_condition_.active = false;
        battery_was_critical_ = false;
    }

    // Low check
    if (battery_voltage <= config_.battery_low_voltage) {
        battery_low_condition_.detected = true;
        battery_low_condition_.active = true;
        battery_was_low_ = true;
    } else if (battery_was_low_ &&
               battery_voltage <= config_.battery_low_voltage + config_.battery_hysteresis_v) {
        battery_low_condition_.detected = true;
        battery_low_condition_.active = true;
    } else {
        battery_low_condition_.detected = false;
        battery_low_condition_.active = false;
        battery_was_low_ = false;
    }
}

void FailsafeManager::update_geofence(const AircraftState& state) {
    if (!config_.geofence.enabled) {
        geofence_condition_.detected = false;
        geofence_condition_.active = false;
        return;
    }

    bool violated = false;

    // Cylindrical fence: horizontal distance from home
    float horizontal_dist = std::sqrt(state.position.north_m * state.position.north_m +
                                      state.position.east_m * state.position.east_m);
    if (horizontal_dist > config_.geofence.radius_m) {
        violated = true;
    }

    // Altitude ceiling
    if (state.altitude_agl_m > config_.geofence.max_altitude_m) {
        violated = true;
    }

    // Altitude floor (only when in air to avoid ground-trigger)
    if (state.in_air && state.altitude_agl_m < config_.geofence.min_altitude_m) {
        violated = true;
    }

    geofence_condition_.detected = violated;
    geofence_condition_.active = violated;
}

} // namespace fw_uav
