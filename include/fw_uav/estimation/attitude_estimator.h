#pragma once

#include <cmath>
#include <cstdint>
#include "fw_uav/core/types.h"
#include "fw_uav/core/result.h"

namespace fw_uav {

// Configuration for the complementary filter attitude estimator
struct AttitudeEstimatorConfig {
    // Complementary filter gain for accel correction (0..1)
    // Higher = trust accelerometer more, lower = trust gyro more
    float accel_gain{0.02f};

    // Complementary filter gain for magnetometer correction (0..1)
    float mag_gain{0.01f};

    // Gyro bias learning rate (0..1). Set to 0 to disable bias estimation.
    float gyro_bias_rate{0.001f};

    // Maximum allowable gyro bias magnitude (rad/s)
    float max_gyro_bias{0.05f};

    // Accelerometer magnitude bounds for valid gravity reference.
    // Reject accel data outside this range (as fraction of 1g).
    float accel_gate_lo{0.8f};   // 0.8g
    float accel_gate_hi{1.2f};   // 1.2g

    // Gravity constant (m/s^2)
    float gravity{9.80665f};
};

// Complementary filter attitude estimator
//
// Fuses high-rate gyroscope integration (short-term accurate, drifts)
// with low-rate accelerometer/magnetometer correction (noisy but drift-free).
//
// The filter operates on quaternions to avoid gimbal lock:
//   1. Predict: integrate gyro to propagate quaternion forward in time.
//   2. Correct roll/pitch: use accelerometer gravity vector to compute
//      a rotation error and apply a fraction (accel_gain) as correction.
//   3. Correct yaw: use magnetometer heading to compute a yaw error
//      and apply a fraction (mag_gain) as correction.
//   4. Optionally estimate gyro bias over time.
class AttitudeEstimator {
public:
    AttitudeEstimator() = default;
    explicit AttitudeEstimator(const AttitudeEstimatorConfig& config);

    // Set configuration
    void set_config(const AttitudeEstimatorConfig& config);
    const AttitudeEstimatorConfig& config() const { return config_; }

    // Update the estimator with new IMU data.
    // @param imu  Current IMU sample (must have gyro_valid=true at minimum)
    // @param dt   Time step in seconds since last call
    // @return Ok on success, error if data is invalid
    Result<void> update(const IMUData& imu, float dt);

    // Get the estimated attitude quaternion (body-to-NED rotation)
    const Quaternion& attitude() const { return attitude_; }

    // Get estimated Euler angles (convenience)
    EulerAngles euler() const { return attitude_.to_euler(); }

    // Get the estimated gyro bias (rad/s, body frame)
    const Vec3f& gyro_bias() const { return gyro_bias_; }

    // Get the bias-corrected angular rate (rad/s, body frame)
    Vec3f corrected_gyro() const { return last_gyro_ - gyro_bias_; }

    // Reset estimator to identity attitude and zero bias
    void reset();

    // Reset to a known attitude
    void reset(const Quaternion& initial_attitude);

    // Initialize attitude from accelerometer + magnetometer (static alignment).
    // Aircraft should be approximately stationary when this is called.
    // @return Ok on success, error if sensor data is unusable
    Result<void> initialize_from_sensors(const IMUData& imu);

    // Whether the estimator has been initialized
    bool initialized() const { return initialized_; }

private:
    // Predict step: integrate gyroscope
    void predict(const Vec3f& gyro, float dt);

    // Correct roll/pitch from accelerometer gravity reference
    void correct_accel(const Vec3f& accel);

    // Correct yaw from magnetometer heading reference
    void correct_mag(const Vec3f& mag);

    // Update gyro bias estimate
    void update_bias(const Vec3f& correction, float dt);

    AttitudeEstimatorConfig config_;
    Quaternion attitude_{Quaternion::identity()};
    Vec3f gyro_bias_{Vec3f::zero()};
    Vec3f last_gyro_{Vec3f::zero()};
    bool initialized_{false};
};

} // namespace fw_uav
