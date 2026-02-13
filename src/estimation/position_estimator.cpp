#include "fw_uav/estimation/position_estimator.h"

namespace fw_uav {

PositionEstimator::PositionEstimator(const PositionEstimatorConfig& config)
    : config_(config) {}

void PositionEstimator::set_config(const PositionEstimatorConfig& config) {
    config_ = config;
}

void PositionEstimator::predict(const Vec3f& accel_ned, float dt) {
    if (dt <= 0.0f) return;

    // Update velocity from acceleration
    velocity_.north_mps += accel_ned.x * dt;
    velocity_.east_mps  += accel_ned.y * dt;
    velocity_.down_mps  += accel_ned.z * dt;

    // Update position from velocity (simple Euler integration)
    position_.north_m += velocity_.north_mps * dt;
    position_.east_m  += velocity_.east_mps  * dt;
    position_.down_m  += velocity_.down_mps  * dt;
}

Result<void> PositionEstimator::update_gps(const GPSData& gps) {
    if (!home_set_) {
        return ErrorCode::NoHomePosition;
    }
    if (!gps.has_fix()) {
        return ErrorCode::GPSNoFix;
    }

    // Convert GPS geo position to NED relative to home
    NEDPosition gps_ned = math::geo_to_ned(gps.position, home_);

    // Complementary filter correction: blend GPS measurement with prediction.
    // position_corrected = predicted + gain * (measured - predicted)
    float pg = config_.gps_pos_gain;
    position_.north_m += pg * (gps_ned.north_m - position_.north_m);
    position_.east_m  += pg * (gps_ned.east_m  - position_.east_m);

    // Vertical: use GPS altitude with a separate (typically lower) gain
    // unless baro is available (baro correction is done in update_baro).
    float ag = config_.gps_alt_gain;
    position_.down_m += ag * (gps_ned.down_m - position_.down_m);

    // Correct velocity from GPS
    float vg = config_.gps_vel_gain;
    velocity_.north_mps += vg * (gps.velocity.north_mps - velocity_.north_mps);
    velocity_.east_mps  += vg * (gps.velocity.east_mps  - velocity_.east_mps);
    velocity_.down_mps  += vg * (gps.velocity.down_mps  - velocity_.down_mps);

    initialized_ = true;
    return Ok();
}

Result<void> PositionEstimator::update_baro(const BarometerData& baro) {
    if (!baro.valid) {
        return ErrorCode::BarometerReadFailed;
    }

    // If no baro reference has been set, silently accept but do nothing useful
    if (!baro_ref_set_) {
        return Ok();
    }

    // Barometer altitude relative to home (positive up)
    float baro_alt_relative = baro.altitude_m - baro_ref_alt_;

    // Convert to NED down (positive down = negative altitude)
    float baro_down = -baro_alt_relative;

    // Complementary filter correction on vertical channel
    float bg = config_.baro_alt_gain;
    position_.down_m += bg * (baro_down - position_.down_m);

    // Derive a vertical velocity correction from the altitude difference.
    // This is a simple first-order approach; a more sophisticated filter
    // would maintain a separate vertical velocity state from baro.
    // We leave the vertical velocity to be primarily corrected by GPS.

    return Ok();
}

void PositionEstimator::set_home(const GeoPosition& home) {
    home_ = home;
    home_set_ = true;
}

void PositionEstimator::set_baro_reference(float baro_alt_m) {
    baro_ref_alt_ = baro_alt_m;
    baro_ref_set_ = true;
}

void PositionEstimator::reset() {
    position_ = NEDPosition();
    velocity_ = NEDVelocity();
    home_ = GeoPosition();
    home_set_ = false;
    baro_ref_alt_ = 0.0f;
    baro_ref_set_ = false;
    initialized_ = false;
}

} // namespace fw_uav
