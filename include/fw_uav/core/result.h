#pragma once

#include <cstdint>
#include <utility>

namespace fw_uav {

// Error codes for the UAV system
enum class ErrorCode : uint16_t {
    Ok = 0,

    // Sensor errors (100-199)
    IMUReadFailed = 100,
    IMUNotCalibrated = 101,
    IMUDataInvalid = 102,
    GPSNoFix = 110,
    GPSTimeout = 111,
    BarometerReadFailed = 120,
    BarometerNotCalibrated = 121,
    AirspeedReadFailed = 130,
    AirspeedNotCalibrated = 131,
    AirspeedBlocked = 132,

    // Navigation errors (200-299)
    NoHomePosition = 200,
    InvalidWaypoint = 201,
    MissionEmpty = 202,
    MissionFull = 203,
    GeofenceViolation = 210,
    GeofenceNotSet = 211,

    // Control errors (300-399)
    ControlNotInitialized = 300,
    InvalidGains = 301,
    InvalidTarget = 302,
    OutputSaturated = 303,

    // System errors (400-499)
    NotArmed = 400,
    AlreadyArmed = 401,
    ArmCheckFailed = 402,
    ModeChangeRejected = 410,
    InvalidParameter = 420,
    ParameterOutOfRange = 421,
    StorageReadFailed = 430,
    StorageWriteFailed = 431,

    // Communication errors (500-599)
    TelemetryTimeout = 500,
    InvalidMessage = 501,
    BufferOverflow = 502,

    // Safety errors (600-699)
    RCLost = 600,
    BatteryLow = 610,
    BatteryCritical = 611,
    FailsafeActive = 620,

    // Generic errors (900-999)
    Unknown = 900,
    NotImplemented = 901,
    Timeout = 902,
    InvalidState = 903,
    NullPointer = 904,
    OutOfMemory = 905
};

// Get a human-readable string for an error code
inline const char* error_code_str(ErrorCode code) {
    switch (code) {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::IMUReadFailed: return "IMU read failed";
        case ErrorCode::IMUNotCalibrated: return "IMU not calibrated";
        case ErrorCode::IMUDataInvalid: return "IMU data invalid";
        case ErrorCode::GPSNoFix: return "No GPS fix";
        case ErrorCode::GPSTimeout: return "GPS timeout";
        case ErrorCode::BarometerReadFailed: return "Barometer read failed";
        case ErrorCode::BarometerNotCalibrated: return "Barometer not calibrated";
        case ErrorCode::AirspeedReadFailed: return "Airspeed read failed";
        case ErrorCode::AirspeedNotCalibrated: return "Airspeed not calibrated";
        case ErrorCode::AirspeedBlocked: return "Airspeed sensor blocked";
        case ErrorCode::NoHomePosition: return "No home position set";
        case ErrorCode::InvalidWaypoint: return "Invalid waypoint";
        case ErrorCode::MissionEmpty: return "Mission is empty";
        case ErrorCode::MissionFull: return "Mission storage full";
        case ErrorCode::GeofenceViolation: return "Geofence violation";
        case ErrorCode::GeofenceNotSet: return "Geofence not set";
        case ErrorCode::ControlNotInitialized: return "Controller not initialized";
        case ErrorCode::InvalidGains: return "Invalid control gains";
        case ErrorCode::InvalidTarget: return "Invalid target value";
        case ErrorCode::OutputSaturated: return "Output saturated";
        case ErrorCode::NotArmed: return "System not armed";
        case ErrorCode::AlreadyArmed: return "System already armed";
        case ErrorCode::ArmCheckFailed: return "Arm check failed";
        case ErrorCode::ModeChangeRejected: return "Mode change rejected";
        case ErrorCode::InvalidParameter: return "Invalid parameter";
        case ErrorCode::ParameterOutOfRange: return "Parameter out of range";
        case ErrorCode::StorageReadFailed: return "Storage read failed";
        case ErrorCode::StorageWriteFailed: return "Storage write failed";
        case ErrorCode::TelemetryTimeout: return "Telemetry timeout";
        case ErrorCode::InvalidMessage: return "Invalid message";
        case ErrorCode::BufferOverflow: return "Buffer overflow";
        case ErrorCode::RCLost: return "RC signal lost";
        case ErrorCode::BatteryLow: return "Battery low";
        case ErrorCode::BatteryCritical: return "Battery critical";
        case ErrorCode::FailsafeActive: return "Failsafe active";
        case ErrorCode::Unknown: return "Unknown error";
        case ErrorCode::NotImplemented: return "Not implemented";
        case ErrorCode::Timeout: return "Timeout";
        case ErrorCode::InvalidState: return "Invalid state";
        case ErrorCode::NullPointer: return "Null pointer";
        case ErrorCode::OutOfMemory: return "Out of memory";
        default: return "Unrecognized error";
    }
}

// Result type for operations that can fail
// Usage:
//   Result<float> calculate_airspeed() {
//       if (sensor_error) return ErrorCode::AirspeedReadFailed;
//       return 25.0f;  // Success
//   }
//
//   auto result = calculate_airspeed();
//   if (result.is_ok()) {
//       float speed = result.value();
//   }
template<typename T>
class Result {
public:
    // Success constructor
    Result(const T& value) : value_(value), error_(ErrorCode::Ok), has_value_(true) {}
    Result(T&& value) : value_(std::move(value)), error_(ErrorCode::Ok), has_value_(true) {}

    // Error constructor
    Result(ErrorCode error) : error_(error), has_value_(false) {}

    // Check if the result is successful
    bool is_ok() const { return has_value_; }
    bool is_error() const { return !has_value_; }
    explicit operator bool() const { return has_value_; }

    // Get the value (undefined behavior if is_error())
    T& value() { return value_; }
    const T& value() const { return value_; }

    // Get the value or a default
    T value_or(const T& default_val) const {
        return has_value_ ? value_ : default_val;
    }

    // Get the error code
    ErrorCode error() const { return error_; }
    const char* error_str() const { return error_code_str(error_); }

    // Unwrap with panic-like behavior (for use when you're certain it's ok)
    T& unwrap() {
        // In production, this would assert or trap
        return value_;
    }

    const T& unwrap() const {
        return value_;
    }

    // Map the value if ok, propagate error otherwise
    template<typename F>
    auto map(F&& f) const -> Result<decltype(f(std::declval<const T&>()))> {
        if (has_value_) {
            return f(value_);
        }
        return error_;
    }

    // Chain operations that return Result
    template<typename F>
    auto and_then(F&& f) const -> decltype(f(std::declval<const T&>())) {
        if (has_value_) {
            return f(value_);
        }
        return error_;
    }

private:
    T value_{};
    ErrorCode error_;
    bool has_value_;
};

// Specialization for void (operations that don't return a value)
template<>
class Result<void> {
public:
    // Success constructor
    Result() : error_(ErrorCode::Ok), has_value_(true) {}

    // Error constructor
    Result(ErrorCode error) : error_(error), has_value_(false) {}

    // Check if the result is successful
    bool is_ok() const { return has_value_; }
    bool is_error() const { return !has_value_; }
    explicit operator bool() const { return has_value_; }

    // Get the error code
    ErrorCode error() const { return error_; }
    const char* error_str() const { return error_code_str(error_); }

    // Create a success result
    static Result ok() { return Result(); }

private:
    ErrorCode error_;
    bool has_value_;
};

// Helper to create success results
template<typename T>
Result<T> Ok(T&& value) {
    return Result<T>(std::forward<T>(value));
}

inline Result<void> Ok() {
    return Result<void>();
}

// Helper to create error results
template<typename T>
Result<T> Err(ErrorCode error) {
    return Result<T>(error);
}

inline Result<void> Err(ErrorCode error) {
    return Result<void>(error);
}

// Macro for early return on error (similar to Rust's ? operator)
#define TRY(expr) \
    do { \
        auto _result = (expr); \
        if (_result.is_error()) { \
            return _result.error(); \
        } \
    } while (0)

// Macro for early return with value extraction
#define TRY_VAL(var, expr) \
    auto _result_##var = (expr); \
    if (_result_##var.is_error()) { \
        return _result_##var.error(); \
    } \
    auto var = _result_##var.value()

} // namespace fw_uav
