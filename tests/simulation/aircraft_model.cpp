#include "fw_uav/core/types.h"

namespace fw_uav {
namespace sim {

// Simple 6-DOF aircraft model for SITL simulation
// Uses linearized aerodynamic coefficients for a generic fixed-wing UAV
class AircraftModel {
public:
    struct Config {
        float mass_kg{2.5f};
        float wing_area_m2{0.5f};
        float wingspan_m{2.0f};
        float chord_m{0.25f};

        // Inertia tensor (diagonal, principal axes)
        float Ixx{0.05f};
        float Iyy{0.10f};
        float Izz{0.12f};

        // Aerodynamic coefficients
        float CL0{0.25f};
        float CLa{5.5f};    // per radian
        float CLq{7.0f};
        float CLde{0.5f};   // elevator

        float CD0{0.02f};
        float CDa{0.3f};
        float k{0.04f};     // induced drag factor

        float Cm0{0.05f};
        float Cma{-0.5f};
        float Cmq{-10.0f};
        float Cmde{-1.2f};

        float Cyb{-0.4f};
        float Cyp{0.0f};
        float Cyr{0.3f};
        float Cyda{0.0f};
        float Cydr{0.2f};

        float Clb{-0.08f};
        float Clp{-0.5f};
        float Clr{0.1f};
        float Clda{0.2f};
        float Cldr{0.01f};

        float Cnb{0.1f};
        float Cnp{-0.03f};
        float Cnr{-0.12f};
        float Cnda{-0.01f};
        float Cndr{-0.08f};

        float prop_efficiency{0.7f};
        float max_thrust_n{15.0f};
    };

    AircraftModel() = default;
    explicit AircraftModel(const Config& config) : config_(config) {}

    // State accessors
    const Vec3f& position() const { return position_; }
    const Vec3f& velocity() const { return velocity_; }
    const Quaternion& attitude() const { return attitude_; }
    const Vec3f& angular_velocity() const { return omega_; }

    // Step the physics forward by dt seconds
    void step(const ControlSurfaces& controls, float dt) {
        // TODO: Implement 6-DOF equations of motion
        // 1. Compute aerodynamic forces and moments
        // 2. Add gravity and thrust
        // 3. Integrate accelerations to get velocities
        // 4. Integrate velocities to get position/attitude

        (void)controls;
        (void)dt;
    }

    // Reset to initial conditions
    void reset() {
        position_ = Vec3f::zero();
        velocity_ = Vec3f(15.0f, 0.0f, 0.0f);  // 15 m/s forward
        attitude_ = Quaternion::identity();
        omega_ = Vec3f::zero();
    }

private:
    Config config_;

    // State
    Vec3f position_{};      // NED position (m)
    Vec3f velocity_{};      // Body-frame velocity (m/s)
    Quaternion attitude_{}; // Body to NED rotation
    Vec3f omega_{};         // Body-frame angular velocity (rad/s)
};

} // namespace sim
} // namespace fw_uav
