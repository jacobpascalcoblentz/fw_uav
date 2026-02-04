#pragma once

#include <cmath>

namespace fw_uav {

// Configuration for a PID controller
struct PIDConfig {
    float kp{0.0f};           // Proportional gain
    float ki{0.0f};           // Integral gain
    float kd{0.0f};           // Derivative gain
    float ff{0.0f};           // Feedforward gain
    float output_min{-1.0f};  // Minimum output
    float output_max{1.0f};   // Maximum output
    float integral_max{1.0f}; // Maximum integral term (anti-windup)
    float d_filter_hz{20.0f}; // Derivative low-pass filter cutoff

    // Create a simple P controller
    static PIDConfig P(float kp, float out_min = -1.0f, float out_max = 1.0f) {
        return {kp, 0.0f, 0.0f, 0.0f, out_min, out_max, 1.0f, 20.0f};
    }

    // Create a PI controller
    static PIDConfig PI(float kp, float ki, float out_min = -1.0f, float out_max = 1.0f) {
        return {kp, ki, 0.0f, 0.0f, out_min, out_max, 1.0f, 20.0f};
    }

    // Create a PID controller
    static PIDConfig PID(float kp, float ki, float kd, float out_min = -1.0f, float out_max = 1.0f) {
        return {kp, ki, kd, 0.0f, out_min, out_max, 1.0f, 20.0f};
    }
};

// PID controller with anti-windup, derivative filtering, and feedforward
class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDConfig& config) : config_(config) {}

    // Set new configuration
    void set_config(const PIDConfig& config) {
        config_ = config;
        // Recalculate filter coefficient if cutoff changed
        update_filter_alpha();
    }

    const PIDConfig& config() const { return config_; }

    // Update the controller with new measurement
    // Returns the control output
    // @param setpoint: Target value
    // @param measurement: Current value
    // @param dt: Time step in seconds
    // @param feedforward: Optional feedforward input (multiplied by ff gain)
    float update(float setpoint, float measurement, float dt, float feedforward = 0.0f) {
        if (dt <= 0.0f) return last_output_;

        float error = setpoint - measurement;

        // Proportional term
        float p_term = config_.kp * error;

        // Integral term with anti-windup
        integral_ += error * dt;
        integral_ = constrain(integral_, -config_.integral_max / config_.ki,
                              config_.integral_max / config_.ki);
        float i_term = config_.ki * integral_;

        // Derivative term (on measurement to avoid derivative kick)
        float derivative = 0.0f;
        if (dt > 0.0f && initialized_) {
            float raw_derivative = -(measurement - last_measurement_) / dt;
            // Low-pass filter the derivative
            derivative = filter_alpha_ * raw_derivative + (1.0f - filter_alpha_) * filtered_derivative_;
            filtered_derivative_ = derivative;
        }
        float d_term = config_.kd * derivative;

        // Feedforward term
        float ff_term = config_.ff * feedforward;

        // Total output
        float output = p_term + i_term + d_term + ff_term;

        // Output limiting
        output = constrain(output, config_.output_min, config_.output_max);

        // Back-calculation anti-windup: reduce integral if output is saturated
        if (config_.ki > 0.0f) {
            float output_unsat = p_term + i_term + d_term + ff_term;
            if (output_unsat != output) {
                // Output was saturated, reduce integral
                integral_ -= (output_unsat - output) / config_.ki;
            }
        }

        // Store state
        last_measurement_ = measurement;
        last_error_ = error;
        last_output_ = output;
        initialized_ = true;

        return output;
    }

    // Update with rate setpoint (for cascaded controllers)
    // In this mode, setpoint is a rate and we measure rate directly
    float update_rate(float rate_setpoint, float rate_measurement, float dt) {
        return update(rate_setpoint, rate_measurement, dt);
    }

    // Reset the controller state
    void reset() {
        integral_ = 0.0f;
        filtered_derivative_ = 0.0f;
        last_measurement_ = 0.0f;
        last_error_ = 0.0f;
        last_output_ = 0.0f;
        initialized_ = false;
    }

    // Reset integral only (useful when mode changes)
    void reset_integral() {
        integral_ = 0.0f;
    }

    // Set integral to a specific value (for bumpless transfer)
    void set_integral(float value) {
        integral_ = value;
    }

    // Get current integral value
    float integral() const { return integral_; }

    // Get last error
    float last_error() const { return last_error_; }

    // Get last output
    float last_output() const { return last_output_; }

    // Check if controller is initialized
    bool initialized() const { return initialized_; }

    // Set the sample rate for filter coefficient calculation
    void set_sample_rate(float hz) {
        sample_rate_hz_ = hz;
        update_filter_alpha();
    }

private:
    static float constrain(float val, float min_val, float max_val) {
        return val < min_val ? min_val : (val > max_val ? max_val : val);
    }

    void update_filter_alpha() {
        if (sample_rate_hz_ > 0.0f && config_.d_filter_hz > 0.0f) {
            // Calculate alpha for first-order low-pass filter
            // alpha = dt / (RC + dt) where RC = 1 / (2*pi*fc)
            float dt = 1.0f / sample_rate_hz_;
            float rc = 1.0f / (2.0f * 3.14159265f * config_.d_filter_hz);
            filter_alpha_ = dt / (rc + dt);
        } else {
            filter_alpha_ = 1.0f;  // No filtering
        }
    }

    PIDConfig config_;

    // State
    float integral_{0.0f};
    float filtered_derivative_{0.0f};
    float last_measurement_{0.0f};
    float last_error_{0.0f};
    float last_output_{0.0f};
    bool initialized_{false};

    // Filter state
    float filter_alpha_{1.0f};
    float sample_rate_hz_{400.0f};  // Default 400 Hz
};

// Cascaded PID (angle -> rate -> output)
// Used for attitude control where we want to control angle but actuate rate
class CascadedPID {
public:
    CascadedPID() = default;
    CascadedPID(const PIDConfig& outer_config, const PIDConfig& inner_config)
        : outer_(outer_config), inner_(inner_config) {}

    void set_outer_config(const PIDConfig& config) { outer_.set_config(config); }
    void set_inner_config(const PIDConfig& config) { inner_.set_config(config); }

    const PIDConfig& outer_config() const { return outer_.config(); }
    const PIDConfig& inner_config() const { return inner_.config(); }

    // Update cascaded controller
    // @param angle_setpoint: Target angle
    // @param angle_measurement: Current angle
    // @param rate_measurement: Current angular rate
    // @param dt: Time step
    // @param feedforward: Optional feedforward to inner loop
    float update(float angle_setpoint, float angle_measurement,
                 float rate_measurement, float dt, float feedforward = 0.0f) {
        // Outer loop: angle -> rate setpoint
        float rate_setpoint = outer_.update(angle_setpoint, angle_measurement, dt);

        // Inner loop: rate setpoint -> output
        return inner_.update(rate_setpoint, rate_measurement, dt, feedforward);
    }

    void reset() {
        outer_.reset();
        inner_.reset();
    }

    void reset_integral() {
        outer_.reset_integral();
        inner_.reset_integral();
    }

    void set_sample_rate(float hz) {
        outer_.set_sample_rate(hz);
        inner_.set_sample_rate(hz);
    }

    PIDController& outer() { return outer_; }
    PIDController& inner() { return inner_; }

private:
    PIDController outer_;
    PIDController inner_;
};

} // namespace fw_uav
