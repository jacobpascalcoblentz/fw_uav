#pragma once

#include <cmath>
#include <cstdint>
#include "fw_uav/core/types.h"
#include "fw_uav/core/result.h"

namespace fw_uav {

// Configuration for the complementary filter position estimator
struct PositionEstimatorConfig {
    // Complementary filter gain for GPS horizontal position correction (0..1)
    // Higher = trust GPS more on each update
    float gps_pos_gain{0.5f};

    // Complementary filter gain for GPS velocity correction (0..1)
    float gps_vel_gain{0.5f};

    // Complementary filter gain for barometer altitude correction (0..1)
    float baro_alt_gain{0.8f};

    // Complementary filter gain for GPS altitude (used when baro unavailable)
    float gps_alt_gain{0.3f};

    // Maximum age of GPS data before it is considered stale (microseconds)
    uint64_t gps_timeout_us{2000000};   // 2 seconds

    // Maximum age of baro data before it is considered stale (microseconds)
    uint64_t baro_timeout_us{1000000};  // 1 second

    // Gravity constant (m/s^2)
    float gravity{9.80665f};
};

// Simple complementary filter position/velocity estimator
//
// Blends GPS position and velocity with barometer altitude to produce
// a smooth NED position and velocity estimate relative to a home position.
//
// Update flow:
//   1. predict(): Propagate position forward using current velocity estimate
//      and optionally body-frame accelerometer data rotated into NED.
//   2. update_gps(): When a new GPS fix arrives, correct horizontal position
//      and all three velocity components.
//   3. update_baro(): When a new baro reading arrives, correct the vertical
//      (down) position using the barometer altitude.
//
// The home position must be set before GPS updates can produce meaningful
// NED coordinates. Typically, home is set once on first GPS fix.
class PositionEstimator {
public:
    PositionEstimator() = default;
    explicit PositionEstimator(const PositionEstimatorConfig& config);

    // Set configuration
    void set_config(const PositionEstimatorConfig& config);
    const PositionEstimatorConfig& config() const { return config_; }

    // Predict step: propagate position using current velocity and optional
    // NED-frame acceleration (gravity-compensated).
    // @param accel_ned  Acceleration in NED frame (m/s^2), gravity removed.
    //                   Pass Vec3f::zero() if not available.
    // @param dt         Time step in seconds
    void predict(const Vec3f& accel_ned, float dt);

    // Correct with a new GPS measurement.
    // Requires home_position to be set.
    // @return Ok on success, error if no home or no fix
    Result<void> update_gps(const GPSData& gps);

    // Correct vertical position with barometer altitude.
    // @return Ok on success, error if baro data invalid
    Result<void> update_baro(const BarometerData& baro);

    // Set the home (reference) position for NED conversion.
    void set_home(const GeoPosition& home);
    bool has_home() const { return home_set_; }
    const GeoPosition& home() const { return home_; }

    // Set the barometer reference altitude (altitude at home position).
    // Called once to zero the baro offset.
    void set_baro_reference(float baro_alt_m);

    // Get estimated NED position relative to home
    const NEDPosition& position() const { return position_; }

    // Get estimated NED velocity
    const NEDVelocity& velocity() const { return velocity_; }

    // Get estimated altitude above home (positive up, meters)
    float altitude() const { return -position_.down_m; }

    // Reset all state
    void reset();

    // Whether the estimator has produced at least one corrected estimate
    bool initialized() const { return initialized_; }

private:
    PositionEstimatorConfig config_;

    NEDPosition position_;
    NEDVelocity velocity_;

    GeoPosition home_;
    bool home_set_{false};

    float baro_ref_alt_{0.0f};
    bool baro_ref_set_{false};

    bool initialized_{false};
};

} // namespace fw_uav
