#include "fw_uav/estimation/attitude_estimator.h"

namespace fw_uav {

AttitudeEstimator::AttitudeEstimator(const AttitudeEstimatorConfig& config)
    : config_(config) {}

void AttitudeEstimator::set_config(const AttitudeEstimatorConfig& config) {
    config_ = config;
}

Result<void> AttitudeEstimator::update(const IMUData& imu, float dt) {
    if (dt <= 0.0f) {
        return ErrorCode::InvalidParameter;
    }
    if (!imu.gyro_valid) {
        return ErrorCode::IMUDataInvalid;
    }
    if (!initialized_) {
        // Auto-initialize from sensors if we have accel data
        if (imu.accel_valid) {
            auto result = initialize_from_sensors(imu);
            if (result.is_error()) {
                return result.error();
            }
        } else {
            // No accel: start from identity
            initialized_ = true;
        }
    }

    last_gyro_ = imu.gyro_radps;

    // 1. Predict: integrate bias-corrected gyro
    Vec3f corrected = imu.gyro_radps - gyro_bias_;
    predict(corrected, dt);

    // 2. Correct roll/pitch from accelerometer
    Vec3f accel_correction = Vec3f::zero();
    if (imu.accel_valid) {
        float accel_mag = imu.accel_mps2.norm() / config_.gravity;
        if (accel_mag >= config_.accel_gate_lo && accel_mag <= config_.accel_gate_hi) {
            correct_accel(imu.accel_mps2);
            // We compute the accel correction vector for bias learning below.
            // The correction is already applied inside correct_accel; here we
            // recompute a lightweight proxy for the bias update.
            Vec3f accel_norm = imu.accel_mps2.normalized();
            // Expected gravity in body frame: rotate NED down (0,0,1) into body
            Vec3f gravity_body = attitude_.conjugate().rotate(Vec3f(0.0f, 0.0f, 1.0f));
            accel_correction = accel_norm.cross(gravity_body);
        }
    }

    // 3. Correct yaw from magnetometer
    if (imu.mag_valid) {
        correct_mag(imu.mag_gauss);
    }

    // 4. Update gyro bias estimate
    if (config_.gyro_bias_rate > 0.0f) {
        update_bias(accel_correction, dt);
    }

    // Re-normalize quaternion to prevent drift
    attitude_ = attitude_.normalized();

    return Ok();
}

void AttitudeEstimator::predict(const Vec3f& gyro, float dt) {
    // First-order quaternion integration:
    //   q_dot = 0.5 * q * omega_quat
    // where omega_quat = (0, wx, wy, wz)
    //
    // q(t+dt) = q(t) + q_dot * dt, then normalize

    float half_dt = 0.5f * dt;
    Quaternion omega_q(0.0f, gyro.x, gyro.y, gyro.z);
    Quaternion q_dot = attitude_ * omega_q;

    attitude_.w += q_dot.w * half_dt;
    attitude_.x += q_dot.x * half_dt;
    attitude_.y += q_dot.y * half_dt;
    attitude_.z += q_dot.z * half_dt;
}

void AttitudeEstimator::correct_accel(const Vec3f& accel) {
    // Normalize the accelerometer reading to get the measured "up" direction
    // in body frame (accel reads +g when stationary, pointing up).
    Vec3f accel_norm = accel.normalized();

    // Expected gravity direction in body frame:
    // In NED, gravity is (0, 0, +g). The body-frame gravity vector is
    // obtained by rotating the NED gravity into the body frame using
    // the conjugate (inverse) of the attitude quaternion.
    Vec3f gravity_body = attitude_.conjugate().rotate(Vec3f(0.0f, 0.0f, 1.0f));

    // The cross product gives an error vector whose direction is the axis
    // of rotation needed to align the measured gravity with the expected,
    // and whose magnitude is proportional to sin(error_angle).
    Vec3f error = accel_norm.cross(gravity_body);

    // Apply a small fraction of this correction as a rotation
    float half_gain = 0.5f * config_.accel_gain;
    Quaternion correction(1.0f, error.x * half_gain, error.y * half_gain, error.z * half_gain);
    attitude_ = (attitude_ * correction).normalized();
}

void AttitudeEstimator::correct_mag(const Vec3f& mag) {
    // Project magnetometer into NED frame
    Vec3f mag_ned = attitude_.rotate(mag);

    // We only care about the horizontal heading component
    float heading_measured = std::atan2(mag_ned.y, mag_ned.x);

    // Expected heading from current attitude
    EulerAngles euler = attitude_.to_euler();
    float heading_expected = euler.yaw;

    // Yaw error
    float yaw_error = math::wrap_pi(heading_measured - heading_expected);

    // Apply correction about the NED down axis (body z-axis after rotation).
    // Rotate the NED z-axis into the body frame to get the correction axis.
    Vec3f z_body = attitude_.conjugate().rotate(Vec3f(0.0f, 0.0f, 1.0f));

    float half_correction = 0.5f * config_.mag_gain * yaw_error;
    Quaternion correction(1.0f,
                          z_body.x * half_correction,
                          z_body.y * half_correction,
                          z_body.z * half_correction);
    attitude_ = (attitude_ * correction).normalized();
}

void AttitudeEstimator::update_bias(const Vec3f& correction, float dt) {
    // Slowly learn gyro bias from the accel correction signal.
    // The correction vector points in the direction the gyro is drifting.
    float rate = config_.gyro_bias_rate * dt;
    gyro_bias_.x += math::constrain(correction.x * rate, -config_.max_gyro_bias, config_.max_gyro_bias);
    gyro_bias_.y += math::constrain(correction.y * rate, -config_.max_gyro_bias, config_.max_gyro_bias);
    gyro_bias_.z += math::constrain(correction.z * rate, -config_.max_gyro_bias, config_.max_gyro_bias);

    // Clamp total bias magnitude
    float bias_mag = gyro_bias_.norm();
    if (bias_mag > config_.max_gyro_bias) {
        gyro_bias_ = gyro_bias_ * (config_.max_gyro_bias / bias_mag);
    }
}

void AttitudeEstimator::reset() {
    attitude_ = Quaternion::identity();
    gyro_bias_ = Vec3f::zero();
    last_gyro_ = Vec3f::zero();
    initialized_ = false;
}

void AttitudeEstimator::reset(const Quaternion& initial_attitude) {
    attitude_ = initial_attitude.normalized();
    gyro_bias_ = Vec3f::zero();
    last_gyro_ = Vec3f::zero();
    initialized_ = true;
}

Result<void> AttitudeEstimator::initialize_from_sensors(const IMUData& imu) {
    if (!imu.accel_valid) {
        return ErrorCode::IMUDataInvalid;
    }

    // Compute roll and pitch from accelerometer (assumes stationary or near)
    Vec3f a = imu.accel_mps2.normalized();

    float pitch = std::asin(math::constrain(-a.x, -1.0f, 1.0f));
    float roll = std::atan2(a.y, a.z);

    float yaw = 0.0f;
    if (imu.mag_valid) {
        // Tilt-compensated heading from magnetometer
        float cos_roll = std::cos(roll);
        float sin_roll = std::sin(roll);
        float cos_pitch = std::cos(pitch);
        float sin_pitch = std::sin(pitch);

        float mx = imu.mag_gauss.x;
        float my = imu.mag_gauss.y;
        float mz = imu.mag_gauss.z;

        // Rotate mag into horizontal plane
        float mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
        float mag_y = my * cos_roll - mz * sin_roll;

        yaw = std::atan2(-mag_y, mag_x);
    }

    attitude_ = Quaternion::from_euler(EulerAngles(roll, pitch, yaw));
    initialized_ = true;
    return Ok();
}

} // namespace fw_uav
