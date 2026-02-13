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
        if (dt <= 0.0f) return;

        // Body-frame velocities (u = forward, v = right, w = down)
        float u = velocity_.x;
        float v = velocity_.y;
        float w = velocity_.z;

        // Total airspeed
        float V = velocity_.norm();
        if (V < 0.1f) V = 0.1f;  // Avoid division by zero

        // Aerodynamic angles
        float alpha = std::atan2(w, u);  // Angle of attack
        float beta = std::asin(v / V);   // Sideslip angle

        // Angular rates (body frame)
        float p = omega_.x;  // Roll rate
        float q = omega_.y;  // Pitch rate
        float r = omega_.z;  // Yaw rate

        // Normalized angular rates
        float p_hat = config_.wingspan_m * p / (2.0f * V);
        float q_hat = config_.chord_m * q / (2.0f * V);
        float r_hat = config_.wingspan_m * r / (2.0f * V);

        // Control surface deflections (convert from normalized to radians)
        // Note: We negate elevator because our convention is +elevator = nose up,
        // but aerodynamic convention is +deflection = nose down (Cmde is negative)
        float de = -controls.elevator * 0.35f;  // ~20 degrees max
        float da = controls.aileron * 0.35f;
        float dr = controls.rudder * 0.35f;

        // ---- Aerodynamic coefficients ----

        // Lift coefficient
        float CL = config_.CL0 + config_.CLa * alpha + config_.CLq * q_hat + config_.CLde * de;

        // Drag coefficient (with induced drag)
        float CD = config_.CD0 + config_.CDa * std::fabs(alpha) + config_.k * CL * CL;

        // Side force coefficient
        float CY = config_.Cyb * beta + config_.Cyp * p_hat + config_.Cyr * r_hat +
                   config_.Cyda * da + config_.Cydr * dr;

        // Roll moment coefficient
        float Cl = config_.Clb * beta + config_.Clp * p_hat + config_.Clr * r_hat +
                   config_.Clda * da + config_.Cldr * dr;

        // Pitch moment coefficient
        float Cm = config_.Cm0 + config_.Cma * alpha + config_.Cmq * q_hat + config_.Cmde * de;

        // Yaw moment coefficient
        float Cn = config_.Cnb * beta + config_.Cnp * p_hat + config_.Cnr * r_hat +
                   config_.Cnda * da + config_.Cndr * dr;

        // ---- Aerodynamic forces and moments ----

        // Dynamic pressure
        constexpr float rho = 1.225f;  // Sea level air density
        float q_bar = 0.5f * rho * V * V;

        // Reference values
        float S = config_.wing_area_m2;
        float b = config_.wingspan_m;
        float c = config_.chord_m;

        // Lift and drag (in stability axes)
        float L = q_bar * S * CL;  // Lift
        float D = q_bar * S * CD;  // Drag
        float Y = q_bar * S * CY;  // Side force

        // Convert stability-axis forces to body-axis forces
        float cos_alpha = std::cos(alpha);
        float sin_alpha = std::sin(alpha);

        float Fx_aero = -D * cos_alpha + L * sin_alpha;  // Along body x (forward)
        float Fy_aero = Y;                                // Along body y (right)
        float Fz_aero = -D * sin_alpha - L * cos_alpha;  // Along body z (down)

        // Moments in body frame
        float L_moment = q_bar * S * b * Cl;  // Rolling moment
        float M_moment = q_bar * S * c * Cm;  // Pitching moment
        float N_moment = q_bar * S * b * Cn;  // Yawing moment

        // ---- Thrust ----
        float thrust = controls.throttle * config_.max_thrust_n * config_.prop_efficiency;
        Fx_aero += thrust;

        // ---- Gravity ----
        // Transform gravity from NED to body frame
        constexpr float g = 9.81f;
        Vec3f gravity_ned(0.0f, 0.0f, g);  // Gravity points down in NED
        Vec3f gravity_body = attitude_.conjugate().rotate(gravity_ned);

        // ---- Total forces ----
        Vec3f F_body(Fx_aero + config_.mass_kg * gravity_body.x,
                     Fy_aero + config_.mass_kg * gravity_body.y,
                     Fz_aero + config_.mass_kg * gravity_body.z);

        // ---- Linear accelerations (body frame) ----
        Vec3f accel_body = F_body / config_.mass_kg;

        // Account for rotating reference frame: a = dv/dt + omega x v
        Vec3f omega_cross_v = omega_.cross(velocity_);

        // ---- Angular accelerations ----
        // Euler's equations: I * omega_dot + omega x (I * omega) = tau
        Vec3f tau(L_moment, M_moment, N_moment);

        Vec3f I_omega(config_.Ixx * omega_.x, config_.Iyy * omega_.y, config_.Izz * omega_.z);
        Vec3f omega_cross_I_omega = omega_.cross(I_omega);

        Vec3f omega_dot((tau.x - omega_cross_I_omega.x) / config_.Ixx,
                        (tau.y - omega_cross_I_omega.y) / config_.Iyy,
                        (tau.z - omega_cross_I_omega.z) / config_.Izz);

        // ---- Integration ----

        // Update angular velocity
        omega_ += omega_dot * dt;

        // Update body-frame velocity
        velocity_ += (accel_body - omega_cross_v) * dt;

        // Update attitude quaternion: q_dot = 0.5 * q * omega
        Quaternion omega_quat(0.0f, omega_.x, omega_.y, omega_.z);
        Quaternion q_dot = attitude_ * omega_quat;
        attitude_.w += 0.5f * q_dot.w * dt;
        attitude_.x += 0.5f * q_dot.x * dt;
        attitude_.y += 0.5f * q_dot.y * dt;
        attitude_.z += 0.5f * q_dot.z * dt;
        attitude_ = attitude_.normalized();

        // Update position (transform body velocity to NED)
        Vec3f velocity_ned = attitude_.rotate(velocity_);
        position_ += velocity_ned * dt;
    }

    // Reset to initial conditions
    void reset() {
        position_ = Vec3f::zero();
        velocity_ = Vec3f(15.0f, 0.0f, 0.0f);  // 15 m/s forward
        attitude_ = Quaternion::identity();
        omega_ = Vec3f::zero();
    }

    // Set initial attitude from euler angles
    void set_attitude(const EulerAngles& euler) {
        attitude_ = Quaternion::from_euler(euler);
    }

    // Set initial position
    void set_position(const Vec3f& pos) {
        position_ = pos;
    }

    // Set initial velocity (body frame)
    void set_velocity(const Vec3f& vel) {
        velocity_ = vel;
    }

    // Get true acceleration in body frame (for IMU simulation)
    Vec3f true_accel_body() const {
        // Specific force = acceleration - gravity (what accelerometer measures)
        constexpr float g = 9.81f;
        Vec3f gravity_ned(0.0f, 0.0f, g);
        Vec3f gravity_body = attitude_.conjugate().rotate(gravity_ned);

        // For steady flight, this approximation works
        // Full implementation would track actual acceleration
        return gravity_body * -1.0f;
    }

    // Get true NED velocity
    Vec3f true_velocity_ned() const {
        return attitude_.rotate(velocity_);
    }

    // Get airspeed
    float airspeed() const {
        return velocity_.norm();
    }

    // Get altitude (negative of down position in NED)
    float altitude() const {
        return -position_.z;
    }

    // Get euler angles
    EulerAngles euler() const {
        return attitude_.to_euler();
    }

    // Get config for reference
    const Config& config() const { return config_; }

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
