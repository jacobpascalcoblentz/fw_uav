#pragma once

#include <cmath>
#include <cstdint>

namespace fw_uav {

// Forward declarations
struct Vec3f;
struct Quaternion;
struct EulerAngles;

// 3D Vector with single precision
struct Vec3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3f() = default;
    constexpr Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Arithmetic operators
    constexpr Vec3f operator+(const Vec3f& v) const { return {x + v.x, y + v.y, z + v.z}; }
    constexpr Vec3f operator-(const Vec3f& v) const { return {x - v.x, y - v.y, z - v.z}; }
    constexpr Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3f operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3f operator-() const { return {-x, -y, -z}; }

    Vec3f& operator+=(const Vec3f& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3f& operator-=(const Vec3f& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3f& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    Vec3f& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    // Vector operations
    constexpr float dot(const Vec3f& v) const { return x * v.x + y * v.y + z * v.z; }

    constexpr Vec3f cross(const Vec3f& v) const {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }

    float norm() const { return std::sqrt(x * x + y * y + z * z); }
    constexpr float norm_squared() const { return x * x + y * y + z * z; }

    Vec3f normalized() const {
        float n = norm();
        if (n > 1e-9f) {
            return *this / n;
        }
        return {0.0f, 0.0f, 0.0f};
    }

    // Element access
    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }

    static constexpr Vec3f zero() { return {0.0f, 0.0f, 0.0f}; }
    static constexpr Vec3f unit_x() { return {1.0f, 0.0f, 0.0f}; }
    static constexpr Vec3f unit_y() { return {0.0f, 1.0f, 0.0f}; }
    static constexpr Vec3f unit_z() { return {0.0f, 0.0f, 1.0f}; }
};

constexpr Vec3f operator*(float s, const Vec3f& v) { return v * s; }

// Euler angles in radians (roll, pitch, yaw - ZYX convention)
struct EulerAngles {
    float roll{0.0f};   // Rotation about X axis (body frame)
    float pitch{0.0f};  // Rotation about Y axis (body frame)
    float yaw{0.0f};    // Rotation about Z axis (body frame)

    constexpr EulerAngles() = default;
    constexpr EulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}

    // Convert to degrees
    constexpr EulerAngles to_degrees() const {
        constexpr float rad2deg = 180.0f / 3.14159265358979323846f;
        return {roll * rad2deg, pitch * rad2deg, yaw * rad2deg};
    }

    static constexpr EulerAngles from_degrees(float roll_deg, float pitch_deg, float yaw_deg) {
        constexpr float deg2rad = 3.14159265358979323846f / 180.0f;
        return {roll_deg * deg2rad, pitch_deg * deg2rad, yaw_deg * deg2rad};
    }
};

// Quaternion for attitude representation (Hamilton convention: w + xi + yj + zk)
struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quaternion() = default;
    constexpr Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Quaternion multiplication
    constexpr Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    constexpr Quaternion conjugate() const { return {w, -x, -y, -z}; }

    float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    Quaternion normalized() const {
        float n = norm();
        if (n > 1e-9f) {
            return {w / n, x / n, y / n, z / n};
        }
        return identity();
    }

    // Rotate a vector by this quaternion: v' = q * v * q^-1
    Vec3f rotate(const Vec3f& v) const {
        Quaternion qv{0.0f, v.x, v.y, v.z};
        Quaternion result = (*this) * qv * conjugate();
        return {result.x, result.y, result.z};
    }

    // Convert to Euler angles (ZYX convention)
    EulerAngles to_euler() const {
        EulerAngles e;

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        e.roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation) - handle gimbal lock
        float sinp = 2.0f * (w * y - z * x);
        if (std::fabs(sinp) >= 1.0f) {
            e.pitch = std::copysign(3.14159265358979323846f / 2.0f, sinp);
        } else {
            e.pitch = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        e.yaw = std::atan2(siny_cosp, cosy_cosp);

        return e;
    }

    // Create from Euler angles (ZYX convention)
    static Quaternion from_euler(const EulerAngles& e) {
        float cr = std::cos(e.roll * 0.5f);
        float sr = std::sin(e.roll * 0.5f);
        float cp = std::cos(e.pitch * 0.5f);
        float sp = std::sin(e.pitch * 0.5f);
        float cy = std::cos(e.yaw * 0.5f);
        float sy = std::sin(e.yaw * 0.5f);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    // Create from axis-angle representation
    static Quaternion from_axis_angle(const Vec3f& axis, float angle) {
        Vec3f n = axis.normalized();
        float half_angle = angle * 0.5f;
        float s = std::sin(half_angle);
        return {std::cos(half_angle), n.x * s, n.y * s, n.z * s};
    }

    static constexpr Quaternion identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }
};

// GPS fix quality
enum class GPSFixType : uint8_t {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
    DGPS = 3,
    RTKFloat = 4,
    RTKFixed = 5
};

// Geographic position
struct GeoPosition {
    double latitude_deg{0.0};   // Degrees
    double longitude_deg{0.0};  // Degrees
    float altitude_msl_m{0.0f}; // Meters above mean sea level

    constexpr GeoPosition() = default;
    constexpr GeoPosition(double lat, double lon, float alt = 0.0f)
        : latitude_deg(lat), longitude_deg(lon), altitude_msl_m(alt) {}
};

// NED position relative to home
struct NEDPosition {
    float north_m{0.0f};
    float east_m{0.0f};
    float down_m{0.0f};  // Positive down

    constexpr NEDPosition() = default;
    constexpr NEDPosition(float n, float e, float d) : north_m(n), east_m(e), down_m(d) {}

    float altitude_agl() const { return -down_m; }  // Above ground level (negative down = positive up)

    Vec3f to_vec3f() const { return {north_m, east_m, down_m}; }
    static NEDPosition from_vec3f(const Vec3f& v) { return {v.x, v.y, v.z}; }
};

// NED velocity
struct NEDVelocity {
    float north_mps{0.0f};
    float east_mps{0.0f};
    float down_mps{0.0f};

    constexpr NEDVelocity() = default;
    constexpr NEDVelocity(float n, float e, float d) : north_mps(n), east_mps(e), down_mps(d) {}

    float ground_speed() const { return std::sqrt(north_mps * north_mps + east_mps * east_mps); }
    float course_over_ground() const { return std::atan2(east_mps, north_mps); }
    float climb_rate() const { return -down_mps; }  // Positive = climbing

    Vec3f to_vec3f() const { return {north_mps, east_mps, down_mps}; }
};

// IMU data
struct IMUData {
    Vec3f accel_mps2{};     // Accelerometer in body frame (m/s^2)
    Vec3f gyro_radps{};     // Gyroscope in body frame (rad/s)
    Vec3f mag_gauss{};      // Magnetometer in body frame (gauss)
    uint64_t timestamp_us{0};
    bool accel_valid{false};
    bool gyro_valid{false};
    bool mag_valid{false};
};

// GPS data
struct GPSData {
    GeoPosition position{};
    NEDVelocity velocity{};
    GPSFixType fix_type{GPSFixType::NoFix};
    uint8_t satellites{0};
    float hdop{99.9f};
    float vdop{99.9f};
    float horizontal_accuracy_m{999.0f};
    float vertical_accuracy_m{999.0f};
    uint64_t timestamp_us{0};

    bool has_fix() const { return fix_type >= GPSFixType::Fix3D; }
    bool has_good_fix() const { return fix_type >= GPSFixType::Fix3D && hdop < 2.5f; }
};

// Barometer data
struct BarometerData {
    float pressure_pa{101325.0f};  // Pascals
    float temperature_c{25.0f};    // Celsius
    float altitude_m{0.0f};        // Pressure altitude (relative to 1013.25 hPa)
    uint64_t timestamp_us{0};
    bool valid{false};
};

// Airspeed data
struct AirspeedData {
    float differential_pressure_pa{0.0f};  // Pitot-static differential
    float indicated_airspeed_mps{0.0f};    // IAS in m/s
    float temperature_c{25.0f};
    uint64_t timestamp_us{0};
    bool valid{false};
    bool healthy{false};  // Sensor self-test passed
};

// RC input channels
struct RCInput {
    static constexpr int MAX_CHANNELS = 16;

    float channels[MAX_CHANNELS]{};  // Normalized -1.0 to +1.0
    uint16_t channel_count{0};
    bool failsafe{true};
    bool frame_lost{false};
    uint64_t timestamp_us{0};

    // Common channel accessors (assumes standard mapping)
    float roll() const { return channels[0]; }      // Aileron
    float pitch() const { return channels[1]; }     // Elevator
    float throttle() const { return channels[2]; }  // Throttle (0 to 1)
    float yaw() const { return channels[3]; }       // Rudder
    float mode_switch() const { return channels[4]; }
    float aux1() const { return channels[5]; }
    float aux2() const { return channels[6]; }

    bool is_valid() const { return !failsafe && !frame_lost && channel_count >= 4; }
};

// Control surface outputs
struct ControlSurfaces {
    float aileron{0.0f};   // -1.0 (left down) to +1.0 (left up)
    float elevator{0.0f};  // -1.0 (down) to +1.0 (up)
    float rudder{0.0f};    // -1.0 (left) to +1.0 (right)
    float throttle{0.0f};  // 0.0 to 1.0
    float flaps{0.0f};     // 0.0 (retracted) to 1.0 (full)

    // Clamp all outputs to valid ranges
    void clamp() {
        auto clamp_val = [](float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };
        aileron = clamp_val(aileron, -1.0f, 1.0f);
        elevator = clamp_val(elevator, -1.0f, 1.0f);
        rudder = clamp_val(rudder, -1.0f, 1.0f);
        throttle = clamp_val(throttle, 0.0f, 1.0f);
        flaps = clamp_val(flaps, 0.0f, 1.0f);
    }
};

// Complete aircraft state
struct AircraftState {
    // Attitude (body relative to NED)
    Quaternion attitude{};
    EulerAngles euler{};        // Computed from quaternion for convenience
    Vec3f angular_rate_radps{}; // Body-frame angular rates

    // Position and velocity (NED frame, relative to home)
    NEDPosition position{};
    NEDVelocity velocity{};
    GeoPosition geo_position{}; // Absolute position
    GeoPosition home_position{};

    // Airdata
    float airspeed_mps{0.0f};
    float altitude_agl_m{0.0f};  // Above ground level
    float altitude_msl_m{0.0f};  // Above mean sea level

    // Status flags
    bool armed{false};
    bool in_air{false};

    // Timing
    uint64_t timestamp_us{0};

    // Convenience methods
    float ground_speed_mps() const { return velocity.ground_speed(); }
    float course_rad() const { return velocity.course_over_ground(); }
    float climb_rate_mps() const { return velocity.climb_rate(); }
    float bank_angle_rad() const { return euler.roll; }
    float pitch_angle_rad() const { return euler.pitch; }
    float heading_rad() const { return euler.yaw; }
};

// Flight mode enumeration
enum class FlightMode : uint8_t {
    Manual = 0,
    Stabilize,
    FlyByWire,
    Auto,
    RTL,
    Loiter,
    Land,
    Takeoff,
    NumModes
};

inline const char* flight_mode_name(FlightMode mode) {
    switch (mode) {
        case FlightMode::Manual:    return "Manual";
        case FlightMode::Stabilize: return "Stabilize";
        case FlightMode::FlyByWire: return "FlyByWire";
        case FlightMode::Auto:      return "Auto";
        case FlightMode::RTL:       return "RTL";
        case FlightMode::Loiter:    return "Loiter";
        case FlightMode::Land:      return "Land";
        case FlightMode::Takeoff:   return "Takeoff";
        default:                    return "Unknown";
    }
}

// Utility functions
namespace math {

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = PI / 2.0f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// Wrap angle to [-PI, PI]
inline float wrap_pi(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// Wrap angle to [0, 2*PI]
inline float wrap_2pi(float angle) {
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f) angle += TWO_PI;
    return angle;
}

// Constrain value to range
template<typename T>
constexpr T constrain(T val, T min_val, T max_val) {
    return val < min_val ? min_val : (val > max_val ? max_val : val);
}

// Linear interpolation
constexpr float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// Map value from one range to another
constexpr float map_range(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Earth radius in meters
constexpr double EARTH_RADIUS_M = 6371000.0;

// Haversine distance between two geo positions
inline double geo_distance_m(const GeoPosition& a, const GeoPosition& b) {
    double lat1 = a.latitude_deg * DEG_TO_RAD;
    double lat2 = b.latitude_deg * DEG_TO_RAD;
    double dlat = (b.latitude_deg - a.latitude_deg) * DEG_TO_RAD;
    double dlon = (b.longitude_deg - a.longitude_deg) * DEG_TO_RAD;

    double sin_dlat = std::sin(dlat / 2.0);
    double sin_dlon = std::sin(dlon / 2.0);
    double h = sin_dlat * sin_dlat + std::cos(lat1) * std::cos(lat2) * sin_dlon * sin_dlon;

    return 2.0 * EARTH_RADIUS_M * std::asin(std::sqrt(h));
}

// Bearing from position a to position b
inline float geo_bearing_rad(const GeoPosition& a, const GeoPosition& b) {
    double lat1 = a.latitude_deg * DEG_TO_RAD;
    double lat2 = b.latitude_deg * DEG_TO_RAD;
    double dlon = (b.longitude_deg - a.longitude_deg) * DEG_TO_RAD;

    double x = std::sin(dlon) * std::cos(lat2);
    double y = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    return static_cast<float>(std::atan2(x, y));
}

// Convert geo position to NED relative to home
inline NEDPosition geo_to_ned(const GeoPosition& pos, const GeoPosition& home) {
    double home_lat_rad = home.latitude_deg * DEG_TO_RAD;

    double dlat = pos.latitude_deg - home.latitude_deg;
    double dlon = pos.longitude_deg - home.longitude_deg;

    float north = static_cast<float>(dlat * DEG_TO_RAD * EARTH_RADIUS_M);
    float east = static_cast<float>(dlon * DEG_TO_RAD * EARTH_RADIUS_M * std::cos(home_lat_rad));
    float down = -(pos.altitude_msl_m - home.altitude_msl_m);

    return {north, east, down};
}

// Convert NED to geo position relative to home
inline GeoPosition ned_to_geo(const NEDPosition& ned, const GeoPosition& home) {
    double home_lat_rad = home.latitude_deg * DEG_TO_RAD;

    double dlat = ned.north_m / EARTH_RADIUS_M;
    double dlon = ned.east_m / (EARTH_RADIUS_M * std::cos(home_lat_rad));

    return {
        home.latitude_deg + dlat * RAD_TO_DEG,
        home.longitude_deg + dlon * RAD_TO_DEG,
        home.altitude_msl_m - ned.down_m
    };
}

} // namespace math

} // namespace fw_uav
