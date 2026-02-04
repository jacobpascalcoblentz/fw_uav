#include "fw_uav/core/types.h"
#include "fw_uav/control/pid.h"

// Include the simulation components directly (header-only style for now)
#include "aircraft_model.cpp"
#include "sensor_sim.cpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace fw_uav;
using namespace fw_uav::sim;

// Simple attitude stabilization controller
class StabilizeController {
public:
    StabilizeController() {
        // Roll rate controller (higher gains for tighter response)
        roll_rate_pid_.set_config(PIDConfig::PID(1.5f, 0.3f, 0.05f));
        roll_rate_pid_.set_sample_rate(400.0f);

        // Pitch rate controller
        pitch_rate_pid_.set_config(PIDConfig::PID(1.8f, 0.4f, 0.05f));
        pitch_rate_pid_.set_sample_rate(400.0f);

        // Yaw rate controller
        yaw_rate_pid_.set_config(PIDConfig::PID(0.8f, 0.1f, 0.02f));
        yaw_rate_pid_.set_sample_rate(400.0f);

        // Outer loop angle controllers (higher gains for faster response)
        roll_angle_pid_.set_config(PIDConfig::P(6.0f, -3.0f, 3.0f));
        pitch_angle_pid_.set_config(PIDConfig::P(6.0f, -3.0f, 3.0f));
    }

    ControlSurfaces update(const EulerAngles& attitude, const Vec3f& rates,
                           const ControlSurfaces& pilot_input, float dt) {
        ControlSurfaces output;

        // Target attitudes from pilot input (scaled)
        float target_roll = pilot_input.aileron * 0.5f;   // ~30 deg max bank
        // Add pitch trim for level flight (aircraft needs slight pitch up)
        float target_pitch = pilot_input.elevator * 0.2f + pitch_trim_;

        // Outer loop: angle error -> rate setpoint
        float roll_rate_sp = roll_angle_pid_.update(target_roll, attitude.roll, dt);
        float pitch_rate_sp = pitch_angle_pid_.update(target_pitch, attitude.pitch, dt);
        float yaw_rate_sp = pilot_input.rudder * 0.5f;  // Direct rate control for yaw

        // Inner loop: rate error -> surface deflection
        output.aileron = roll_rate_pid_.update(roll_rate_sp, rates.x, dt);
        output.elevator = pitch_rate_pid_.update(pitch_rate_sp, rates.y, dt);
        output.rudder = yaw_rate_pid_.update(yaw_rate_sp, rates.z, dt);
        output.throttle = pilot_input.throttle;

        output.clamp();
        return output;
    }

    void set_pitch_trim(float trim_rad) { pitch_trim_ = trim_rad; }

    void reset() {
        roll_rate_pid_.reset();
        pitch_rate_pid_.reset();
        yaw_rate_pid_.reset();
        roll_angle_pid_.reset();
        pitch_angle_pid_.reset();
    }

private:
    PIDController roll_rate_pid_;
    PIDController pitch_rate_pid_;
    PIDController yaw_rate_pid_;
    PIDController roll_angle_pid_;
    PIDController pitch_angle_pid_;
    float pitch_trim_{0.05f};  // Small pitch-up trim for level flight (~3 deg)
};

// Visualization hook (can be extended for external visualization)
struct SimVisualization {
    bool enabled{false};
    int print_interval_steps{400};  // Print every N steps (1 second at 400Hz)

    void update(int step, float sim_time, const AircraftModel& aircraft,
                const ControlSurfaces& controls) {
        if (!enabled) return;
        if (step % print_interval_steps != 0) return;

        EulerAngles euler = aircraft.euler();
        Vec3f pos = aircraft.position();
        float airspeed = aircraft.airspeed();
        float alt = aircraft.altitude();

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "t=" << sim_time << "s | ";
        std::cout << "alt=" << alt << "m | ";
        std::cout << "IAS=" << airspeed << "m/s | ";
        std::cout << "roll=" << euler.roll * math::RAD_TO_DEG << "° | ";
        std::cout << "pitch=" << euler.pitch * math::RAD_TO_DEG << "° | ";
        std::cout << "hdg=" << math::wrap_2pi(euler.yaw) * math::RAD_TO_DEG << "° | ";
        std::cout << "N=" << pos.x << " E=" << pos.y;
        std::cout << "\n";
    }
};

int main(int argc, char* argv[]) {
    std::cout << "FW_UAV Software-in-the-Loop Simulation\n";
    std::cout << "======================================\n\n";

    // Parse command line args
    bool verbose = false;
    float sim_duration = 10.0f;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-v" || arg == "--verbose") {
            verbose = true;
        } else if (arg == "-t" && i + 1 < argc) {
            sim_duration = std::stof(argv[++i]);
        }
    }

    // Simulation parameters
    constexpr float sim_dt = 0.0025f;      // 400 Hz simulation rate
    constexpr float imu_dt = 0.0025f;      // 400 Hz IMU rate
    constexpr float gps_dt = 0.1f;         // 10 Hz GPS rate
    constexpr float baro_dt = 0.02f;       // 50 Hz baro rate

    // Initialize aircraft model
    AircraftModel::Config aircraft_config;
    aircraft_config.mass_kg = 2.5f;
    aircraft_config.max_thrust_n = 15.0f;
    AircraftModel aircraft(aircraft_config);

    // Initialize at 100m altitude, flying north at 15 m/s
    aircraft.reset();
    aircraft.set_position(Vec3f(0.0f, 0.0f, -100.0f));  // 100m altitude (NED: down is positive)
    aircraft.set_velocity(Vec3f(15.0f, 0.0f, 0.0f));    // 15 m/s forward

    // Initialize sensor simulation
    SensorSim::NoiseConfig noise_config;
    SensorSim sensors(noise_config);
    sensors.randomize_biases();

    // Initialize controller
    StabilizeController controller;

    // Initialize visualization
    SimVisualization viz;
    viz.enabled = verbose;
    viz.print_interval_steps = static_cast<int>(1.0f / sim_dt);  // Every 1 second

    // Simulated home position (arbitrary location)
    GeoPosition home_position(37.7749, -122.4194, 100.0f);  // San Francisco, 100m MSL

    std::cout << "Simulation parameters:\n";
    std::cout << "  Physics rate: " << 1.0f / sim_dt << " Hz\n";
    std::cout << "  Duration: " << sim_duration << " s\n";
    std::cout << "  Aircraft mass: " << aircraft_config.mass_kg << " kg\n";
    std::cout << "  Initial altitude: " << aircraft.altitude() << " m\n";
    std::cout << "  Initial airspeed: " << aircraft.airspeed() << " m/s\n";
    std::cout << "\n";

    if (verbose) {
        std::cout << "Running simulation with telemetry...\n\n";
    }

    // Pilot input (simulated RC)
    ControlSurfaces pilot_input;
    pilot_input.throttle = 0.6f;  // 60% throttle for level flight
    pilot_input.aileron = 0.0f;
    pilot_input.elevator = 0.0f;
    pilot_input.rudder = 0.0f;

    // Timing accumulators
    float imu_accum = 0.0f;
    float gps_accum = 0.0f;
    float baro_accum = 0.0f;

    // Main simulation loop
    float sim_time = 0.0f;
    int steps = 0;
    uint64_t timestamp_us = 0;

    auto wall_start = std::chrono::high_resolution_clock::now();

    while (sim_time < sim_duration) {
        // ---- Generate sensor data ----
        imu_accum += sim_dt;
        gps_accum += sim_dt;
        baro_accum += sim_dt;

        IMUData imu_data;
        if (imu_accum >= imu_dt) {
            imu_accum = 0.0f;
            // Generate IMU data from ground truth
            Vec3f true_accel = aircraft.true_accel_body();
            Vec3f true_gyro = aircraft.angular_velocity();

            // Simple mag model (local magnetic field in body frame)
            Vec3f mag_ned(0.2f, 0.05f, 0.4f);  // Simplified local field
            Vec3f true_mag = aircraft.attitude().conjugate().rotate(mag_ned);

            imu_data = sensors.generate_imu(true_accel, true_gyro, true_mag, timestamp_us);
        }

        GPSData gps_data;
        if (gps_accum >= gps_dt) {
            gps_accum = 0.0f;
            // Convert NED position to geo
            NEDPosition ned = NEDPosition::from_vec3f(aircraft.position());
            GeoPosition geo = math::ned_to_geo(ned, home_position);

            Vec3f vel_ned = aircraft.true_velocity_ned();
            NEDVelocity velocity(vel_ned.x, vel_ned.y, vel_ned.z);

            gps_data = sensors.generate_gps(geo, velocity, timestamp_us);
        }

        BarometerData baro_data;
        if (baro_accum >= baro_dt) {
            baro_accum = 0.0f;
            float altitude_msl = home_position.altitude_msl_m + aircraft.altitude();
            baro_data = sensors.generate_baro(altitude_msl, 15.0f, timestamp_us);
        }

        // ---- Run flight controller ----
        // Use ground truth for now (proper implementation would use estimated state)
        EulerAngles attitude = aircraft.euler();
        Vec3f rates = aircraft.angular_velocity();

        // Introduce a small perturbation at t=2s to test stability
        if (sim_time > 2.0f && sim_time < 2.5f) {
            pilot_input.aileron = 0.3f;  // Command a bank
        } else if (sim_time > 2.5f && sim_time < 5.0f) {
            pilot_input.aileron = 0.0f;  // Release
        } else if (sim_time > 5.0f && sim_time < 5.5f) {
            pilot_input.elevator = 0.2f;  // Pitch up
        } else if (sim_time > 5.5f) {
            pilot_input.elevator = 0.0f;  // Release
        }

        ControlSurfaces actuator_cmd = controller.update(attitude, rates, pilot_input, sim_dt);

        // ---- Step physics model ----
        aircraft.step(actuator_cmd, sim_dt);

        // ---- Visualization ----
        viz.update(steps, sim_time, aircraft, actuator_cmd);

        // ---- Check for simulation termination ----
        // Stop if aircraft crashes (altitude <= 0)
        if (aircraft.altitude() < 0.0f) {
            std::cout << "Aircraft crashed at t=" << sim_time << "s\n";
            break;
        }

        // Stop if aircraft is way too high (simulation diverged)
        if (aircraft.altitude() > 10000.0f) {
            std::cout << "Simulation diverged (altitude > 10km) at t=" << sim_time << "s\n";
            break;
        }

        sim_time += sim_dt;
        steps++;
        timestamp_us += static_cast<uint64_t>(sim_dt * 1e6f);
    }

    auto wall_end = std::chrono::high_resolution_clock::now();
    auto wall_duration = std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start);

    std::cout << "\n======================================\n";
    std::cout << "Simulation complete\n";
    std::cout << "  Simulated time: " << sim_time << " s\n";
    std::cout << "  Wall clock time: " << wall_duration.count() << " ms\n";
    std::cout << "  Realtime factor: " << (sim_time * 1000.0f) / wall_duration.count() << "x\n";
    std::cout << "  Total steps: " << steps << "\n";
    std::cout << "\nFinal state:\n";
    std::cout << "  Altitude: " << aircraft.altitude() << " m\n";
    std::cout << "  Airspeed: " << aircraft.airspeed() << " m/s\n";
    std::cout << "  Position N: " << aircraft.position().x << " m\n";
    std::cout << "  Position E: " << aircraft.position().y << " m\n";

    EulerAngles final_euler = aircraft.euler();
    std::cout << "  Roll: " << final_euler.roll * math::RAD_TO_DEG << " deg\n";
    std::cout << "  Pitch: " << final_euler.pitch * math::RAD_TO_DEG << " deg\n";
    std::cout << "  Heading: " << math::wrap_2pi(final_euler.yaw) * math::RAD_TO_DEG << " deg\n";

    return 0;
}
