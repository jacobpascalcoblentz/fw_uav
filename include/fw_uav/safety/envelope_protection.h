#pragma once

#include <cstdint>

#include "fw_uav/core/result.h"
#include "fw_uav/core/types.h"

namespace fw_uav {

// Bit flags indicating which protections fired during the last apply() call
enum class ProtectionFlags : uint8_t {
    None          = 0,
    StallProtect  = 1 << 0,
    OverspeedProtect = 1 << 1,
    OverbankProtect  = 1 << 2,
    AltFloorProtect  = 1 << 3
};

inline ProtectionFlags operator|(ProtectionFlags a, ProtectionFlags b) {
    return static_cast<ProtectionFlags>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline ProtectionFlags& operator|=(ProtectionFlags& a, ProtectionFlags b) {
    a = a | b;
    return a;
}

inline bool has_flag(ProtectionFlags flags, ProtectionFlags test) {
    return (static_cast<uint8_t>(flags) & static_cast<uint8_t>(test)) != 0;
}

// Configuration for envelope protection limits
struct EnvelopeProtectionConfig {
    // Airspeed limits (m/s)
    float min_airspeed_mps{12.0f};       // Stall speed with margin
    float max_airspeed_mps{40.0f};       // Vne / structural limit

    // Soft thresholds where protection begins to ramp in
    float stall_margin_mps{2.0f};        // Protection starts ramping at min + margin
    float overspeed_margin_mps{3.0f};    // Protection starts ramping at max - margin

    // Bank angle limit (radians)
    float max_bank_angle_rad{1.0472f};   // ~60 degrees

    // Altitude floor (meters AGL)
    float min_altitude_m{15.0f};

    // Throttle overrides
    float stall_throttle_boost{0.8f};    // Minimum throttle during stall protect
    float overspeed_throttle_limit{0.0f}; // Maximum throttle during overspeed protect

    // Pitch limits under protection (radians)
    float stall_max_pitch_up_rad{0.0f};    // Max nose-up pitch when stall protecting (0 = level)
    float overspeed_max_pitch_down_rad{0.0f}; // Max nose-down pitch when overspeed protecting

    // Altitude floor: minimum climb command (m/s equivalent mapped to elevator)
    float alt_floor_min_pitch_rad{0.0873f}; // ~5 degrees nose-up when below floor
};

// Envelope protection system: constrains control surface commands to keep
// the aircraft within safe flight parameters.
class EnvelopeProtection {
public:
    EnvelopeProtection() = default;
    explicit EnvelopeProtection(const EnvelopeProtectionConfig& config);

    // Set or update configuration
    void configure(const EnvelopeProtectionConfig& config);
    const EnvelopeProtectionConfig& config() const { return config_; }

    // Apply envelope protections to the given control surfaces based on
    // the current aircraft state. Surfaces are modified in place.
    // Returns Ok with ProtectionFlags indicating what fired,
    // or an error if state data is insufficient (e.g. no valid airspeed).
    Result<ProtectionFlags> apply(ControlSurfaces& surfaces,
                                  const AircraftState& state) const;

    // Query methods for individual protections (stateless, based on state)
    bool is_near_stall(const AircraftState& state) const;
    bool is_near_overspeed(const AircraftState& state) const;
    bool is_overbanked(const AircraftState& state) const;
    bool is_below_floor(const AircraftState& state) const;

private:
    // Individual protection applications. Each returns true if it modified
    // the surfaces.
    bool apply_stall_protection(ControlSurfaces& surfaces,
                                const AircraftState& state) const;
    bool apply_overspeed_protection(ControlSurfaces& surfaces,
                                    const AircraftState& state) const;
    bool apply_overbank_protection(ControlSurfaces& surfaces,
                                   const AircraftState& state) const;
    bool apply_altitude_floor(ControlSurfaces& surfaces,
                              const AircraftState& state) const;

    EnvelopeProtectionConfig config_{};
};

} // namespace fw_uav
