#include "fw_uav/core/types.h"
#include "fw_uav/core/result.h"
#include "fw_uav/control/pid.h"

#include <iostream>

using namespace fw_uav;

// Full autopilot example stub
// Will demonstrate complete flight controller with:
// - Multiple flight modes (Manual, Stabilize, FBW, Auto)
// - Navigation (waypoint following)
// - Altitude/airspeed control (TECS)
// - Failsafe handling

int main() {
    std::cout << "FW_UAV Full Autopilot Example\n";
    std::cout << "==============================\n\n";

    std::cout << "This example demonstrates the complete autopilot architecture.\n";
    std::cout << "Full implementation pending - requires:\n";
    std::cout << "  - TECS (Total Energy Control System)\n";
    std::cout << "  - L1 Navigation Controller\n";
    std::cout << "  - Mission/Waypoint Manager\n";
    std::cout << "  - State Estimator\n";
    std::cout << "  - Failsafe Manager\n\n";

    // Placeholder: show library capabilities
    std::cout << "Library components available:\n";

    // Types
    Vec3f pos(100.0f, 50.0f, -200.0f);
    std::cout << "  Vec3f: position = (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";

    Quaternion q = Quaternion::from_euler(EulerAngles::from_degrees(10.0f, 5.0f, 45.0f));
    EulerAngles e = q.to_euler().to_degrees();
    std::cout << "  Quaternion -> Euler: roll=" << e.roll << ", pitch=" << e.pitch << ", yaw=" << e.yaw << "\n";

    // Result type
    auto result = Ok(42.0f);
    if (result) {
        std::cout << "  Result<float>: " << result.value() << "\n";
    }

    // PID controller
    PIDController pid(PIDConfig::PID(1.0f, 0.1f, 0.01f));
    float output = pid.update(10.0f, 8.0f, 0.01f);
    std::cout << "  PID output: " << output << "\n";

    // GPS data
    GPSData gps;
    gps.position = GeoPosition(37.7749, -122.4194, 100.0f);
    gps.fix_type = GPSFixType::Fix3D;
    std::cout << "  GPS: " << gps.position.latitude_deg << ", " << gps.position.longitude_deg << " ("
              << (gps.has_fix() ? "fix" : "no fix") << ")\n";

    std::cout << "\nAutopilot stub complete. See minimal_stabilize for working example.\n";

    return 0;
}
