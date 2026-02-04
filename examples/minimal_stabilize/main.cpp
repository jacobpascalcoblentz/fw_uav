#include "fw_uav/core/types.h"
#include "fw_uav/core/result.h"
#include "fw_uav/control/pid.h"

#include <iostream>

using namespace fw_uav;

// Minimal stabilize mode example
// Demonstrates basic attitude stabilization using cascaded PID
int main() {
    std::cout << "FW_UAV Minimal Stabilize Example\n";
    std::cout << "=================================\n\n";

    // Configure roll/pitch attitude controllers
    // Outer loop: angle -> rate setpoint
    // Inner loop: rate -> surface deflection

    // Roll controller
    CascadedPID roll_controller(
        PIDConfig::P(4.0f),                              // Angle P gain
        PIDConfig::PID(0.1f, 0.02f, 0.01f, -1.0f, 1.0f)  // Rate PID
    );

    // Pitch controller
    CascadedPID pitch_controller(
        PIDConfig::P(3.0f),
        PIDConfig::PID(0.08f, 0.015f, 0.008f, -1.0f, 1.0f)
    );

    // Yaw rate damper (P only)
    PIDController yaw_damper(PIDConfig::P(0.3f, -1.0f, 1.0f));

    // Simulate a few control cycles
    constexpr float dt = 0.0025f;  // 400 Hz
    constexpr int num_steps = 400; // 1 second

    // Simulated stick inputs (stabilize mode = attitude command)
    float roll_stick = 0.2f;   // 20% right roll command
    float pitch_stick = -0.1f; // 10% nose down command

    // Map stick to attitude targets
    constexpr float max_roll_deg = 45.0f;
    constexpr float max_pitch_deg = 30.0f;
    float roll_target_rad = roll_stick * max_roll_deg * math::DEG_TO_RAD;
    float pitch_target_rad = pitch_stick * max_pitch_deg * math::DEG_TO_RAD;

    std::cout << "Roll target: " << roll_target_rad * math::RAD_TO_DEG << " deg\n";
    std::cout << "Pitch target: " << pitch_target_rad * math::RAD_TO_DEG << " deg\n\n";

    // Simulated aircraft state (starts wings level)
    EulerAngles attitude{0.0f, 0.0f, 0.0f};
    Vec3f angular_rate{0.0f, 0.0f, 0.0f};

    ControlSurfaces outputs;

    for (int i = 0; i < num_steps; i++) {
        // Run attitude controllers
        outputs.aileron = roll_controller.update(
            roll_target_rad, attitude.roll,
            angular_rate.x, dt
        );

        outputs.elevator = pitch_controller.update(
            pitch_target_rad, attitude.pitch,
            angular_rate.y, dt
        );

        outputs.rudder = yaw_damper.update(0.0f, angular_rate.z, dt);

        outputs.clamp();

        // Simple fake dynamics for demo
        angular_rate.x += outputs.aileron * 2.0f * dt;
        angular_rate.y += outputs.elevator * 1.5f * dt;
        attitude.roll += angular_rate.x * dt;
        attitude.pitch += angular_rate.y * dt;

        // Print progress every 100 steps
        if (i % 100 == 0) {
            std::cout << "Step " << i << ": "
                      << "roll=" << attitude.roll * math::RAD_TO_DEG << " deg, "
                      << "pitch=" << attitude.pitch * math::RAD_TO_DEG << " deg, "
                      << "ail=" << outputs.aileron << ", "
                      << "ele=" << outputs.elevator << "\n";
        }
    }

    std::cout << "\nFinal state:\n";
    std::cout << "  Roll: " << attitude.roll * math::RAD_TO_DEG << " deg\n";
    std::cout << "  Pitch: " << attitude.pitch * math::RAD_TO_DEG << " deg\n";
    std::cout << "  Aileron: " << outputs.aileron << "\n";
    std::cout << "  Elevator: " << outputs.elevator << "\n";

    return 0;
}
