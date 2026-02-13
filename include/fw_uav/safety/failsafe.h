#pragma once

#include <cstdint>

#include "fw_uav/core/result.h"
#include "fw_uav/core/types.h"

namespace fw_uav {

// Priority levels for failsafe conditions (higher value = higher priority)
enum class FailsafePriority : uint8_t {
    None = 0,
    GPSLoss = 1,
    BatteryLow = 2,
    RCLoss = 3,
    GeofenceViolation = 4,
    BatteryCritical = 5
};

// Individual failsafe condition status
struct FailsafeCondition {
    bool detected{false};
    bool active{false};          // True once timeout/threshold has been exceeded
    float elapsed_s{0.0f};       // Time since condition first detected
    FailsafePriority priority{FailsafePriority::None};
    FlightMode recommended_mode{FlightMode::RTL};
};

// Actions recommended by the failsafe system after an update cycle
struct FailsafeAction {
    bool failsafe_active{false};
    FlightMode recommended_mode{FlightMode::Manual};
    FailsafePriority highest_priority{FailsafePriority::None};

    // Individual condition flags
    bool rc_lost{false};
    bool gps_lost{false};
    bool battery_low{false};
    bool battery_critical{false};
    bool geofence_violated{false};
};

// Cylindrical geofence centered on home position
struct GeofenceConfig {
    bool enabled{false};
    float radius_m{500.0f};        // Horizontal radius from home
    float max_altitude_m{200.0f};  // Maximum altitude AGL
    float min_altitude_m{10.0f};   // Minimum altitude AGL (while in air)
};

// Configuration for all failsafe thresholds
struct FailsafeConfig {
    // RC loss
    float rc_loss_timeout_s{1.5f};         // Seconds without valid RC before failsafe

    // GPS loss
    float gps_loss_timeout_s{5.0f};        // Seconds without valid GPS fix before failsafe

    // Battery
    float battery_low_voltage{11.1f};      // Volts (3S LiPo: 3.7V/cell)
    float battery_critical_voltage{10.2f}; // Volts (3S LiPo: 3.4V/cell)
    float battery_hysteresis_v{0.3f};      // Voltage hysteresis to prevent flapping

    // Geofence
    GeofenceConfig geofence{};

    // Flight mode overrides for each condition
    FlightMode rc_loss_mode{FlightMode::RTL};
    FlightMode gps_loss_mode{FlightMode::Loiter};
    FlightMode battery_low_mode{FlightMode::RTL};
    FlightMode battery_critical_mode{FlightMode::Land};
    FlightMode geofence_violation_mode{FlightMode::RTL};
};

// Failsafe manager: evaluates sensor inputs against thresholds and
// recommends flight-mode transitions when safety conditions are detected.
class FailsafeManager {
public:
    FailsafeManager() = default;
    explicit FailsafeManager(const FailsafeConfig& config);

    // Set or update configuration
    void configure(const FailsafeConfig& config);
    const FailsafeConfig& config() const { return config_; }

    // Main update method. Call once per control loop iteration.
    //   state          - current aircraft state
    //   rc             - latest RC input
    //   battery_voltage - current battery voltage in volts
    //   dt_s           - time step in seconds
    // Returns the combined failsafe action reflecting the highest-priority
    // active condition.
    FailsafeAction update(const AircraftState& state,
                          const RCInput& rc,
                          float battery_voltage,
                          float dt_s);

    // Reset all failsafe conditions (e.g. after disarm)
    void reset();

    // Query individual condition status
    const FailsafeCondition& rc_condition() const { return rc_condition_; }
    const FailsafeCondition& gps_condition() const { return gps_condition_; }
    const FailsafeCondition& battery_low_condition() const { return battery_low_condition_; }
    const FailsafeCondition& battery_critical_condition() const { return battery_critical_condition_; }
    const FailsafeCondition& geofence_condition() const { return geofence_condition_; }

    // Check if any failsafe is currently active
    bool any_active() const;

private:
    void update_rc(const RCInput& rc, float dt_s);
    void update_gps(const AircraftState& state, float dt_s);
    void update_battery(float battery_voltage);
    void update_geofence(const AircraftState& state);

    FailsafeConfig config_{};

    FailsafeCondition rc_condition_{};
    FailsafeCondition gps_condition_{};
    FailsafeCondition battery_low_condition_{};
    FailsafeCondition battery_critical_condition_{};
    FailsafeCondition geofence_condition_{};

    // Track whether battery was previously in a low/critical state
    // to apply hysteresis on recovery
    bool battery_was_low_{false};
    bool battery_was_critical_{false};
};

} // namespace fw_uav
