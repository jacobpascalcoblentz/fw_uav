#include "fw_uav/core/types.h"
#include <random>

namespace fw_uav {
namespace sim {

// Simulated sensor suite for SITL
// Adds realistic noise, bias, and latency to ground-truth state
class SensorSim {
public:
    struct NoiseConfig {
        // IMU noise
        float accel_noise_mps2{0.02f};
        float accel_bias_mps2{0.05f};
        float gyro_noise_radps{0.001f};
        float gyro_bias_radps{0.01f};
        float mag_noise_gauss{0.005f};

        // GPS noise
        float gps_pos_noise_m{1.5f};
        float gps_vel_noise_mps{0.1f};
        int gps_latency_ms{100};

        // Barometer noise
        float baro_noise_pa{10.0f};
        float baro_bias_pa{50.0f};

        // Airspeed noise
        float airspeed_noise_pa{5.0f};
        float airspeed_bias_pa{10.0f};
    };

    SensorSim() : rng_(std::random_device{}()) {}
    explicit SensorSim(const NoiseConfig& config) : config_(config), rng_(std::random_device{}()) {}

    // Generate simulated IMU data from ground truth state
    IMUData generate_imu(const Vec3f& true_accel_body,
                         const Vec3f& true_gyro_body,
                         const Vec3f& true_mag_body,
                         uint64_t timestamp_us) {
        IMUData imu;

        // Add noise and bias to accelerometer
        imu.accel_mps2.x = true_accel_body.x + accel_bias_.x + noise(config_.accel_noise_mps2);
        imu.accel_mps2.y = true_accel_body.y + accel_bias_.y + noise(config_.accel_noise_mps2);
        imu.accel_mps2.z = true_accel_body.z + accel_bias_.z + noise(config_.accel_noise_mps2);

        // Add noise and bias to gyroscope
        imu.gyro_radps.x = true_gyro_body.x + gyro_bias_.x + noise(config_.gyro_noise_radps);
        imu.gyro_radps.y = true_gyro_body.y + gyro_bias_.y + noise(config_.gyro_noise_radps);
        imu.gyro_radps.z = true_gyro_body.z + gyro_bias_.z + noise(config_.gyro_noise_radps);

        // Add noise to magnetometer
        imu.mag_gauss.x = true_mag_body.x + noise(config_.mag_noise_gauss);
        imu.mag_gauss.y = true_mag_body.y + noise(config_.mag_noise_gauss);
        imu.mag_gauss.z = true_mag_body.z + noise(config_.mag_noise_gauss);

        imu.timestamp_us = timestamp_us;
        imu.accel_valid = true;
        imu.gyro_valid = true;
        imu.mag_valid = true;

        return imu;
    }

    // Generate simulated GPS data from ground truth state
    GPSData generate_gps(const GeoPosition& true_position,
                         const NEDVelocity& true_velocity,
                         uint64_t timestamp_us) {
        GPSData gps;

        // Add position noise
        gps.position.latitude_deg = true_position.latitude_deg +
            noise(config_.gps_pos_noise_m) / 111320.0;  // ~111km per degree
        gps.position.longitude_deg = true_position.longitude_deg +
            noise(config_.gps_pos_noise_m) / 111320.0;
        gps.position.altitude_msl_m = true_position.altitude_msl_m +
            noise(config_.gps_pos_noise_m * 1.5f);

        // Add velocity noise
        gps.velocity.north_mps = true_velocity.north_mps + noise(config_.gps_vel_noise_mps);
        gps.velocity.east_mps = true_velocity.east_mps + noise(config_.gps_vel_noise_mps);
        gps.velocity.down_mps = true_velocity.down_mps + noise(config_.gps_vel_noise_mps);

        gps.fix_type = GPSFixType::Fix3D;
        gps.satellites = 12;
        gps.hdop = 1.0f;
        gps.vdop = 1.5f;
        gps.horizontal_accuracy_m = config_.gps_pos_noise_m;
        gps.vertical_accuracy_m = config_.gps_pos_noise_m * 1.5f;
        gps.timestamp_us = timestamp_us;

        return gps;
    }

    // Generate simulated barometer data from altitude
    BarometerData generate_baro(float true_altitude_m,
                                float temperature_c,
                                uint64_t timestamp_us) {
        BarometerData baro;

        // Convert altitude to pressure (simplified)
        float pressure = 101325.0f * std::pow(1.0f - true_altitude_m / 44330.0f, 5.255f);

        baro.pressure_pa = pressure + baro_bias_ + noise(config_.baro_noise_pa);
        baro.temperature_c = temperature_c + noise(0.5f);
        baro.altitude_m = true_altitude_m + noise(config_.baro_noise_pa / 12.0f);
        baro.timestamp_us = timestamp_us;
        baro.valid = true;

        return baro;
    }

    // Generate simulated airspeed data from true airspeed
    AirspeedData generate_airspeed(float true_airspeed_mps,
                                   float temperature_c,
                                   uint64_t timestamp_us) {
        AirspeedData airspeed;

        // Convert airspeed to differential pressure
        // dp = 0.5 * rho * v^2
        constexpr float rho = 1.225f;
        float true_dp = 0.5f * rho * true_airspeed_mps * true_airspeed_mps;

        airspeed.differential_pressure_pa = true_dp + airspeed_bias_ +
            noise(config_.airspeed_noise_pa);
        if (airspeed.differential_pressure_pa < 0.0f) {
            airspeed.differential_pressure_pa = 0.0f;
        }

        // Recalculate IAS from noisy differential pressure
        airspeed.indicated_airspeed_mps =
            std::sqrt(2.0f * airspeed.differential_pressure_pa / rho);
        airspeed.temperature_c = temperature_c;
        airspeed.timestamp_us = timestamp_us;
        airspeed.valid = true;
        airspeed.healthy = true;

        return airspeed;
    }

    // Initialize random sensor biases
    void randomize_biases() {
        accel_bias_.x = noise(config_.accel_bias_mps2);
        accel_bias_.y = noise(config_.accel_bias_mps2);
        accel_bias_.z = noise(config_.accel_bias_mps2);

        gyro_bias_.x = noise(config_.gyro_bias_radps);
        gyro_bias_.y = noise(config_.gyro_bias_radps);
        gyro_bias_.z = noise(config_.gyro_bias_radps);

        baro_bias_ = noise(config_.baro_bias_pa);
        airspeed_bias_ = noise(config_.airspeed_bias_pa);
    }

private:
    float noise(float stddev) {
        std::normal_distribution<float> dist(0.0f, stddev);
        return dist(rng_);
    }

    NoiseConfig config_;
    std::mt19937 rng_;

    // Sensor biases (constant per session)
    Vec3f accel_bias_{};
    Vec3f gyro_bias_{};
    float baro_bias_{0.0f};
    float airspeed_bias_{0.0f};
};

} // namespace sim
} // namespace fw_uav
