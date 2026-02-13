#pragma once

#include <cmath>
#include "fw_uav/core/types.h"
#include "fw_uav/control/pid.h"

namespace fw_uav {

// Configuration for the Total Energy Control System
struct TECSConfig {
    // Airspeed limits (m/s)
    float min_airspeed_mps{12.0f};
    float max_airspeed_mps{30.0f};

    // Climb/sink rate limits (m/s)
    float max_climb_rate_mps{3.0f};
    float max_sink_rate_mps{3.0f};     // Positive value (applied as negative)

    // Pitch angle limits (radians)
    float max_pitch_rad{0.436f};       // ~25 degrees
    float min_pitch_rad{-0.262f};      // ~-15 degrees

    // Throttle limits (0.0 to 1.0)
    float min_throttle{0.0f};
    float max_throttle{1.0f};
    float cruise_throttle{0.5f};       // Nominal cruise throttle

    // Energy controller gains
    float total_energy_kp{1.0f};       // Throttle response to total energy error
    float total_energy_ki{0.1f};       // Integral gain for total energy
    float energy_balance_kp{1.0f};     // Pitch response to energy balance error
    float energy_balance_ki{0.1f};     // Integral gain for energy balance

    // Speed weight: 0.0 = pure altitude priority, 2.0 = pure speed priority
    // 1.0 = equal priority
    float speed_weight{1.0f};

    // Derivative filter cutoff (Hz) for STE and SBE rate estimation
    float rate_filter_hz{0.5f};

    // Time constant for coordinated response (seconds)
    float time_constant_s{5.0f};

    // Integral limits
    float throttle_integral_max{0.3f};
    float pitch_integral_max{0.1f};

    // Standard factory configurations
    static TECSConfig Default() {
        return {};
    }

    static TECSConfig HighPerformance() {
        TECSConfig c;
        c.max_climb_rate_mps = 5.0f;
        c.max_sink_rate_mps = 5.0f;
        c.total_energy_kp = 1.5f;
        c.energy_balance_kp = 1.5f;
        c.time_constant_s = 3.0f;
        return c;
    }

    static TECSConfig Conservative() {
        TECSConfig c;
        c.max_climb_rate_mps = 2.0f;
        c.max_sink_rate_mps = 2.0f;
        c.total_energy_kp = 0.7f;
        c.energy_balance_kp = 0.7f;
        c.time_constant_s = 7.0f;
        return c;
    }
};

// Diagnostic output from TECS update
struct TECSOutput {
    float throttle{0.0f};             // Throttle command (0.0 to 1.0)
    float pitch_ref_rad{0.0f};        // Pitch reference (radians)

    // Diagnostic values
    float total_energy_error{0.0f};   // STE error (normalized)
    float energy_balance_error{0.0f}; // SBE error (normalized)
    float total_energy_rate{0.0f};    // STE rate (normalized)
    float energy_balance_rate{0.0f};  // SBE rate (normalized)
    float speed_error_mps{0.0f};      // Airspeed error
    float altitude_error_m{0.0f};     // Altitude error
};

// Total Energy Control System
//
// TECS coordinates throttle and pitch to simultaneously control airspeed
// and altitude. It works on the principle that:
//   - Total Specific Energy (STE) = kinetic + potential = V^2/(2g) + h
//   - Throttle controls total energy rate (both speed and altitude)
//   - Pitch controls energy distribution (trade speed for altitude)
//
// Reference: Lambregts, A.A., "Vertical Flight Path and Speed Control
// Autopilot Design Using Total Energy Principles", AIAA-83-2239
class TECSController {
public:
    TECSController() = default;
    explicit TECSController(const TECSConfig& config) : config_(config) {}

    void set_config(const TECSConfig& config) {
        config_ = config;
    }

    const TECSConfig& config() const { return config_; }

    // Update TECS controller
    // @param airspeed_setpoint_mps: Desired airspeed (m/s)
    // @param altitude_setpoint_m: Desired altitude (m, MSL)
    // @param airspeed_mps: Current airspeed (m/s)
    // @param altitude_m: Current altitude (m, MSL)
    // @param climb_rate_mps: Current vertical speed (m/s, positive up)
    // @param dt: Time step (seconds)
    // @return TECSOutput with throttle and pitch commands
    TECSOutput update(float airspeed_setpoint_mps, float altitude_setpoint_m,
                      float airspeed_mps, float altitude_m,
                      float climb_rate_mps, float dt) {
        if (dt <= 0.0f) return last_output_;

        // Constrain setpoints
        float spd_sp = math::constrain(airspeed_setpoint_mps,
                                       config_.min_airspeed_mps,
                                       config_.max_airspeed_mps);

        // Constrain current airspeed (avoid division by zero in energy calcs)
        float spd = math::constrain(airspeed_mps,
                                    config_.min_airspeed_mps * 0.5f,
                                    config_.max_airspeed_mps * 1.5f);

        // --- Specific Energy Calculations ---
        // Normalize energies by dividing by g (makes units: m for PE, m for KE)
        // Specific kinetic energy: V^2 / (2g)
        // Specific potential energy: h
        constexpr float g = 9.80665f;
        constexpr float inv_2g = 1.0f / (2.0f * g);

        float ske = spd * spd * inv_2g;
        float spe = altitude_m;
        float ske_sp = spd_sp * spd_sp * inv_2g;
        float spe_sp = altitude_setpoint_m;

        // --- Specific Energy Rates ---
        // SKE rate = V * V_dot / g  (approximated from airspeed change)
        // SPE rate = h_dot (climb rate)
        float ske_rate = 0.0f;
        if (initialized_) {
            float accel = (spd - last_airspeed_mps_) / dt;
            ske_rate = spd * accel / g;
        }
        float spe_rate = climb_rate_mps;

        // --- Rate Setpoints from Position/Speed Errors ---
        // Compute desired rate of change to close the error
        float spe_error = spe_sp - spe;
        float ske_error = ske_sp - ske;

        // Limit desired climb rate
        float desired_climb = spe_error / config_.time_constant_s;
        desired_climb = math::constrain(desired_climb,
                                        -config_.max_sink_rate_mps,
                                        config_.max_climb_rate_mps);
        float spe_rate_sp = desired_climb;

        // Limit desired acceleration (expressed as energy rate)
        float desired_accel_energy = ske_error / config_.time_constant_s;
        // Limit based on max achievable acceleration
        float max_accel_energy = config_.max_climb_rate_mps;
        desired_accel_energy = math::constrain(desired_accel_energy,
                                               -max_accel_energy,
                                               max_accel_energy);
        float ske_rate_sp = desired_accel_energy;

        // --- Total Specific Energy (STE) and Balance (SBE) ---
        // STE = SKE + SPE    (total energy - controlled by throttle)
        // SBE = SPE - SKE    (energy distribution - controlled by pitch)
        float ste_rate_sp = ske_rate_sp + spe_rate_sp;
        float ste_rate = ske_rate + spe_rate;

        // Apply speed_weight to prioritize speed vs altitude
        // weight = 0: pure altitude, weight = 2: pure speed
        float w = math::constrain(config_.speed_weight, 0.0f, 2.0f);
        float sbe_rate_sp = spe_rate_sp - ske_rate_sp * w;
        float sbe_rate = spe_rate - ske_rate * w;

        // --- Rate Error Filtering ---
        float ste_rate_error = ste_rate_sp - ste_rate;
        float sbe_rate_error = sbe_rate_sp - sbe_rate;

        // Low-pass filter the rate errors
        if (initialized_) {
            float alpha = compute_filter_alpha(dt);
            ste_rate_error = alpha * ste_rate_error + (1.0f - alpha) * last_ste_rate_error_;
            sbe_rate_error = alpha * sbe_rate_error + (1.0f - alpha) * last_sbe_rate_error_;
        }

        // --- Integral Terms ---
        ste_integral_ += ste_rate_error * dt;
        ste_integral_ = math::constrain(ste_integral_,
                                        -config_.throttle_integral_max / config_.total_energy_ki,
                                        config_.throttle_integral_max / config_.total_energy_ki);

        sbe_integral_ += sbe_rate_error * dt;
        sbe_integral_ = math::constrain(sbe_integral_,
                                        -config_.pitch_integral_max / config_.energy_balance_ki,
                                        config_.pitch_integral_max / config_.energy_balance_ki);

        // --- Throttle Command ---
        // Throttle controls total energy rate
        float throttle_p = config_.total_energy_kp * ste_rate_error;
        float throttle_i = config_.total_energy_ki * ste_integral_;
        float throttle = config_.cruise_throttle + throttle_p + throttle_i;
        throttle = math::constrain(throttle, config_.min_throttle, config_.max_throttle);

        // Back-calculation anti-windup for throttle integral
        float throttle_unsat = config_.cruise_throttle + throttle_p + throttle_i;
        if (throttle_unsat != throttle && config_.total_energy_ki > 0.0f) {
            ste_integral_ -= (throttle_unsat - throttle) / config_.total_energy_ki;
        }

        // --- Pitch Command ---
        // Pitch controls energy balance (trade speed for altitude)
        float pitch_p = config_.energy_balance_kp * sbe_rate_error;
        float pitch_i = config_.energy_balance_ki * sbe_integral_;
        float pitch_ref = pitch_p + pitch_i;
        pitch_ref = math::constrain(pitch_ref, config_.min_pitch_rad, config_.max_pitch_rad);

        // Back-calculation anti-windup for pitch integral
        float pitch_unsat = pitch_p + pitch_i;
        if (pitch_unsat != pitch_ref && config_.energy_balance_ki > 0.0f) {
            sbe_integral_ -= (pitch_unsat - pitch_ref) / config_.energy_balance_ki;
        }

        // --- Store State ---
        last_airspeed_mps_ = spd;
        last_ste_rate_error_ = ste_rate_error;
        last_sbe_rate_error_ = sbe_rate_error;
        initialized_ = true;

        // --- Build Output ---
        TECSOutput output;
        output.throttle = throttle;
        output.pitch_ref_rad = pitch_ref;
        output.total_energy_error = ste_rate_error;
        output.energy_balance_error = sbe_rate_error;
        output.total_energy_rate = ste_rate;
        output.energy_balance_rate = sbe_rate;
        output.speed_error_mps = spd_sp - spd;
        output.altitude_error_m = spe_error;

        last_output_ = output;
        return output;
    }

    // Reset controller state
    void reset() {
        ste_integral_ = 0.0f;
        sbe_integral_ = 0.0f;
        last_airspeed_mps_ = 0.0f;
        last_ste_rate_error_ = 0.0f;
        last_sbe_rate_error_ = 0.0f;
        initialized_ = false;
        last_output_ = {};
    }

    // Reset integrals only (useful for mode transitions)
    void reset_integrals() {
        ste_integral_ = 0.0f;
        sbe_integral_ = 0.0f;
    }

    // Check if controller has been initialized
    bool initialized() const { return initialized_; }

    // Get last output
    const TECSOutput& last_output() const { return last_output_; }

    // Get integral values for diagnostics
    float ste_integral() const { return ste_integral_; }
    float sbe_integral() const { return sbe_integral_; }

private:
    float compute_filter_alpha(float dt) const {
        if (config_.rate_filter_hz <= 0.0f || dt <= 0.0f) return 1.0f;
        float rc = 1.0f / (2.0f * math::PI * config_.rate_filter_hz);
        return dt / (rc + dt);
    }

    TECSConfig config_;

    // State
    float ste_integral_{0.0f};
    float sbe_integral_{0.0f};
    float last_airspeed_mps_{0.0f};
    float last_ste_rate_error_{0.0f};
    float last_sbe_rate_error_{0.0f};
    bool initialized_{false};

    TECSOutput last_output_{};
};

} // namespace fw_uav
