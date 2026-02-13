#include "fw_uav/safety/envelope_protection.h"

#include <cmath>

namespace fw_uav {

EnvelopeProtection::EnvelopeProtection(const EnvelopeProtectionConfig& config)
    : config_(config) {}

void EnvelopeProtection::configure(const EnvelopeProtectionConfig& config) {
    config_ = config;
}

Result<ProtectionFlags> EnvelopeProtection::apply(ControlSurfaces& surfaces,
                                                   const AircraftState& state) const {
    ProtectionFlags flags = ProtectionFlags::None;

    if (apply_stall_protection(surfaces, state)) {
        flags |= ProtectionFlags::StallProtect;
    }
    if (apply_overspeed_protection(surfaces, state)) {
        flags |= ProtectionFlags::OverspeedProtect;
    }
    if (apply_overbank_protection(surfaces, state)) {
        flags |= ProtectionFlags::OverbankProtect;
    }
    if (apply_altitude_floor(surfaces, state)) {
        flags |= ProtectionFlags::AltFloorProtect;
    }

    surfaces.clamp();

    return flags;
}

// ---- Query methods ---------------------------------------------------------

bool EnvelopeProtection::is_near_stall(const AircraftState& state) const {
    return state.airspeed_mps < (config_.min_airspeed_mps + config_.stall_margin_mps);
}

bool EnvelopeProtection::is_near_overspeed(const AircraftState& state) const {
    return state.airspeed_mps > (config_.max_airspeed_mps - config_.overspeed_margin_mps);
}

bool EnvelopeProtection::is_overbanked(const AircraftState& state) const {
    return std::fabs(state.euler.roll) > config_.max_bank_angle_rad;
}

bool EnvelopeProtection::is_below_floor(const AircraftState& state) const {
    return state.altitude_agl_m < config_.min_altitude_m;
}

// ---- Private protection implementations ------------------------------------

bool EnvelopeProtection::apply_stall_protection(ControlSurfaces& surfaces,
                                                 const AircraftState& state) const {
    // Protection ramps in as airspeed drops below (min + margin) toward min.
    float threshold = config_.min_airspeed_mps + config_.stall_margin_mps;
    if (state.airspeed_mps >= threshold) {
        return false;
    }

    // Compute a blend factor: 0 at threshold, 1 at min_airspeed (or below)
    float range = config_.stall_margin_mps;
    float blend = 0.0f;
    if (range > 0.0f) {
        blend = (threshold - state.airspeed_mps) / range;
        blend = math::constrain(blend, 0.0f, 1.0f);
    } else {
        blend = (state.airspeed_mps < config_.min_airspeed_mps) ? 1.0f : 0.0f;
    }

    // Limit pitch-up (positive elevator = nose up).
    // At full blend, clamp elevator to stall_max_pitch_up_rad mapped to [-1,1].
    // Simple approach: limit elevator to at most the configured ceiling.
    // Since elevator is normalized [-1,1] and we don't know the exact
    // mapping to pitch rate, we limit it directly: at full blend the max
    // elevator is the config value (defaults to 0, i.e. no nose-up allowed).
    float max_elevator = math::lerp(1.0f, config_.stall_max_pitch_up_rad, blend);
    if (surfaces.elevator > max_elevator) {
        surfaces.elevator = max_elevator;
    }

    // Boost throttle: ramp minimum throttle up to stall_throttle_boost
    float min_throttle = blend * config_.stall_throttle_boost;
    if (surfaces.throttle < min_throttle) {
        surfaces.throttle = min_throttle;
    }

    return true;
}

bool EnvelopeProtection::apply_overspeed_protection(ControlSurfaces& surfaces,
                                                     const AircraftState& state) const {
    float threshold = config_.max_airspeed_mps - config_.overspeed_margin_mps;
    if (state.airspeed_mps <= threshold) {
        return false;
    }

    // Blend: 0 at threshold, 1 at max_airspeed (or above)
    float range = config_.overspeed_margin_mps;
    float blend = 0.0f;
    if (range > 0.0f) {
        blend = (state.airspeed_mps - threshold) / range;
        blend = math::constrain(blend, 0.0f, 1.0f);
    } else {
        blend = (state.airspeed_mps > config_.max_airspeed_mps) ? 1.0f : 0.0f;
    }

    // Limit pitch-down (negative elevator = nose down).
    float min_elevator = math::lerp(-1.0f, config_.overspeed_max_pitch_down_rad, blend);
    if (surfaces.elevator < min_elevator) {
        surfaces.elevator = min_elevator;
    }

    // Reduce throttle: at full blend, cap throttle to overspeed limit
    float max_throttle = math::lerp(1.0f, config_.overspeed_throttle_limit, blend);
    if (surfaces.throttle > max_throttle) {
        surfaces.throttle = max_throttle;
    }

    return true;
}

bool EnvelopeProtection::apply_overbank_protection(ControlSurfaces& surfaces,
                                                    const AircraftState& state) const {
    float bank = state.euler.roll;
    float max_bank = config_.max_bank_angle_rad;

    if (std::fabs(bank) <= max_bank) {
        return false;
    }

    // Aircraft is past max bank. Constrain aileron to prevent further banking.
    // If bank is positive (right wing down), prevent further positive aileron
    // (which would roll further right). If bank is negative, prevent further
    // negative aileron.
    if (bank > max_bank) {
        // Over-banked right. Limit aileron to zero or negative (roll left).
        if (surfaces.aileron > 0.0f) {
            surfaces.aileron = 0.0f;
        }
    } else if (bank < -max_bank) {
        // Over-banked left. Limit aileron to zero or positive (roll right).
        if (surfaces.aileron < 0.0f) {
            surfaces.aileron = 0.0f;
        }
    }

    return true;
}

bool EnvelopeProtection::apply_altitude_floor(ControlSurfaces& surfaces,
                                               const AircraftState& state) const {
    if (state.altitude_agl_m >= config_.min_altitude_m) {
        return false;
    }

    // Below altitude floor. Prevent further descent by setting a minimum
    // elevator (nose-up) command. The blend factor increases as altitude
    // decreases toward zero.
    float blend = 0.0f;
    if (config_.min_altitude_m > 0.0f) {
        blend = 1.0f - (state.altitude_agl_m / config_.min_altitude_m);
        blend = math::constrain(blend, 0.0f, 1.0f);
    } else {
        blend = 1.0f;
    }

    // Set minimum elevator to demand pitch-up
    float min_elevator = blend * config_.alt_floor_min_pitch_rad;
    // Note: using alt_floor_min_pitch_rad as a normalized elevator value here.
    // A more sophisticated system would convert pitch angle to elevator
    // deflection through the control law, but for direct surface limiting
    // this is a reasonable first approximation.

    // Here we just ensure elevator doesn't command nose-down when below floor.
    // At full blend, elevator is forced to at least the configured minimum pitch.
    if (surfaces.elevator < min_elevator) {
        surfaces.elevator = min_elevator;
    }

    // Also prevent throttle from being cut entirely
    float min_throttle = blend * 0.3f;
    if (surfaces.throttle < min_throttle) {
        surfaces.throttle = min_throttle;
    }

    return true;
}

} // namespace fw_uav
