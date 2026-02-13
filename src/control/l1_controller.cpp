#include <fw_uav/control/l1_controller.h>

namespace fw_uav {

L1Output L1Controller::navigate_path(const NEDPosition& position_ned,
                                     float ground_speed,
                                     float course_rad,
                                     const NEDPosition& prev_wp,
                                     const NEDPosition& next_wp) {
    output_ = L1Output{};

    if (ground_speed < MIN_GROUND_SPEED) {
        return output_;
    }

    // Vector from prev_wp to next_wp (path direction)
    float path_n = next_wp.north_m - prev_wp.north_m;
    float path_e = next_wp.east_m - prev_wp.east_m;
    float path_len = std::sqrt(path_n * path_n + path_e * path_e);

    if (path_len < 1e-3f) {
        // Degenerate path — fall back to direct-to
        return navigate_waypoint(position_ned, ground_speed, course_rad, next_wp);
    }

    // Unit vector along path
    float path_un = path_n / path_len;
    float path_ue = path_e / path_len;

    // Vector from prev_wp to aircraft
    float ac_n = position_ned.north_m - prev_wp.north_m;
    float ac_e = position_ned.east_m - prev_wp.east_m;

    // Along-track distance (projection of ac onto path)
    float along_track = ac_n * path_un + ac_e * path_ue;

    // Crosstrack error (positive = aircraft right of path)
    float crosstrack = ac_e * path_un - ac_n * path_ue;

    // L1 distance
    float l1_dist = config_.l1_distance(ground_speed);

    // Clamp L1 distance to a minimum
    if (l1_dist < 1.0f) {
        l1_dist = 1.0f;
    }

    // Compute the L1 reference point along the path ahead of closest point
    // The L1 point is on the path, at the intersection of the L1 circle
    // centered on the aircraft with the path line.
    //
    // We project the aircraft position onto the path, then advance by
    // sqrt(L1^2 - crosstrack^2) if the crosstrack error is within L1.
    // If crosstrack > L1, we aim at the closest point on the path.
    float l1_advance;
    float ct_abs = std::fabs(crosstrack);
    if (ct_abs < l1_dist) {
        l1_advance = std::sqrt(l1_dist * l1_dist - crosstrack * crosstrack);
    } else {
        l1_advance = 0.0f;
    }

    // L1 reference point along the path
    float ref_along = along_track + l1_advance;

    // Clamp reference point to the path segment
    if (ref_along < 0.0f) ref_along = 0.0f;
    if (ref_along > path_len) ref_along = path_len;

    // L1 reference point in NED
    float ref_n = prev_wp.north_m + path_un * ref_along;
    float ref_e = prev_wp.east_m + path_ue * ref_along;

    // Vector from aircraft to L1 reference point
    float l1_vec_n = ref_n - position_ned.north_m;
    float l1_vec_e = ref_e - position_ned.east_m;

    // Bearing to L1 reference point
    float bearing_to_ref = std::atan2(l1_vec_e, l1_vec_n);

    // Bearing error: angle between ground track and bearing to L1 point
    float eta = math::wrap_pi(bearing_to_ref - course_rad);

    // L1 lateral acceleration demand:
    // a_cmd = 2 * V^2 / L1 * sin(eta)
    float lateral_accel = 2.0f * ground_speed * ground_speed / l1_dist * std::sin(eta);

    // Distance to next waypoint
    float to_wp_n = next_wp.north_m - position_ned.north_m;
    float to_wp_e = next_wp.east_m - position_ned.east_m;
    float wp_dist = std::sqrt(to_wp_n * to_wp_n + to_wp_e * to_wp_e);

    output_.lateral_accel_mps2 = lateral_accel;
    output_.bank_angle_rad = lateral_accel_to_bank(lateral_accel, ground_speed);
    output_.bearing_error_rad = eta;
    output_.crosstrack_error_m = crosstrack;
    output_.wp_distance_m = wp_dist;
    output_.data_valid = true;

    return output_;
}

L1Output L1Controller::navigate_waypoint(const NEDPosition& position_ned,
                                         float ground_speed,
                                         float course_rad,
                                         const NEDPosition& target_wp) {
    output_ = L1Output{};

    if (ground_speed < MIN_GROUND_SPEED) {
        return output_;
    }

    // Vector from aircraft to waypoint
    float to_wp_n = target_wp.north_m - position_ned.north_m;
    float to_wp_e = target_wp.east_m - position_ned.east_m;
    float wp_dist = std::sqrt(to_wp_n * to_wp_n + to_wp_e * to_wp_e);

    if (wp_dist < 1e-3f) {
        output_.data_valid = true;
        return output_;
    }

    // Bearing to waypoint
    float bearing_to_wp = std::atan2(to_wp_e, to_wp_n);

    // Bearing error
    float eta = math::wrap_pi(bearing_to_wp - course_rad);

    // L1 distance
    float l1_dist = config_.l1_distance(ground_speed);
    if (l1_dist < 1.0f) {
        l1_dist = 1.0f;
    }

    // For direct-to waypoint, use the distance to the waypoint as L1 if closer
    float effective_l1 = (wp_dist < l1_dist) ? wp_dist : l1_dist;
    if (effective_l1 < 1.0f) {
        effective_l1 = 1.0f;
    }

    // L1 lateral acceleration demand
    float lateral_accel = 2.0f * ground_speed * ground_speed / effective_l1 * std::sin(eta);

    output_.lateral_accel_mps2 = lateral_accel;
    output_.bank_angle_rad = lateral_accel_to_bank(lateral_accel, ground_speed);
    output_.bearing_error_rad = eta;
    output_.crosstrack_error_m = 0.0f; // No path to measure crosstrack against
    output_.wp_distance_m = wp_dist;
    output_.data_valid = true;

    return output_;
}

L1Output L1Controller::navigate_loiter(const NEDPosition& position_ned,
                                       float ground_speed,
                                       float course_rad,
                                       const NEDPosition& center,
                                       float radius_m) {
    output_ = L1Output{};

    if (ground_speed < MIN_GROUND_SPEED) {
        return output_;
    }

    float abs_radius = std::fabs(radius_m);
    if (abs_radius < 1.0f) {
        abs_radius = 1.0f;
    }

    // Direction: positive radius = clockwise (right turns), negative = CCW
    float direction = (radius_m >= 0.0f) ? 1.0f : -1.0f;

    // Vector from center to aircraft
    float dx = position_ned.north_m - center.north_m;
    float dy = position_ned.east_m - center.east_m;
    float dist_to_center = std::sqrt(dx * dx + dy * dy);

    if (dist_to_center < 1e-3f) {
        // On top of center — nudge outward along current heading
        dx = std::cos(course_rad);
        dy = std::sin(course_rad);
        dist_to_center = 1.0f;
    }

    // L1 distance
    float l1_dist = config_.l1_distance(ground_speed);
    if (l1_dist < 1.0f) {
        l1_dist = 1.0f;
    }

    // Bearing from center to aircraft
    float bearing_from_center = std::atan2(dy, dx);

    // Target point on the orbit: advance along the circle by a fraction
    // proportional to the L1 distance. This gives the tangent-pursuit behavior.
    //
    // Angular offset from current radial to the L1 target on the orbit:
    // For an aircraft on the orbit: omega = L1 / (2 * R)
    // Clamped to avoid asin domain errors when far from orbit.
    float orbit_frac = l1_dist / (2.0f * abs_radius);
    if (orbit_frac > 1.0f) orbit_frac = 1.0f;
    float omega = std::asin(orbit_frac) * direction;

    // L1 target point on the orbit
    float target_bearing = bearing_from_center + omega;
    float target_n = center.north_m + abs_radius * std::cos(target_bearing);
    float target_e = center.east_m + abs_radius * std::sin(target_bearing);

    // Vector from aircraft to target point
    float to_target_n = target_n - position_ned.north_m;
    float to_target_e = target_e - position_ned.east_m;

    // Bearing to L1 target
    float bearing_to_target = std::atan2(to_target_e, to_target_n);

    // Bearing error
    float eta = math::wrap_pi(bearing_to_target - course_rad);

    // Compute effective distance to target for L1 formula
    float dist_to_target = std::sqrt(to_target_n * to_target_n + to_target_e * to_target_e);
    float effective_l1 = (dist_to_target > 1.0f) ? dist_to_target : 1.0f;

    // L1 lateral acceleration demand
    float lateral_accel = 2.0f * ground_speed * ground_speed / effective_l1 * std::sin(eta);

    // When well-established on the orbit, add centripetal acceleration
    // to improve tracking. Blend based on distance from orbit.
    float radial_error = dist_to_center - abs_radius;
    float blend = 1.0f - math::constrain(std::fabs(radial_error) / abs_radius, 0.0f, 1.0f);
    float centripetal = direction * ground_speed * ground_speed / abs_radius;
    lateral_accel += blend * centripetal;

    // Crosstrack error relative to the orbit (positive = outside)
    float crosstrack = direction * radial_error;

    output_.lateral_accel_mps2 = lateral_accel;
    output_.bank_angle_rad = lateral_accel_to_bank(lateral_accel, ground_speed);
    output_.bearing_error_rad = eta;
    output_.crosstrack_error_m = crosstrack;
    output_.wp_distance_m = dist_to_center;
    output_.data_valid = true;

    return output_;
}

bool L1Controller::has_reached_waypoint(const NEDPosition& position_ned,
                                        const NEDPosition& prev_wp,
                                        const NEDPosition& next_wp) const {
    // Vector from prev_wp to next_wp
    float path_n = next_wp.north_m - prev_wp.north_m;
    float path_e = next_wp.east_m - prev_wp.east_m;

    // Vector from next_wp to aircraft
    float to_ac_n = position_ned.north_m - next_wp.north_m;
    float to_ac_e = position_ned.east_m - next_wp.east_m;

    // Dot product: positive means aircraft has passed the waypoint
    return (path_n * to_ac_n + path_e * to_ac_e) > 0.0f;
}

void L1Controller::reset() {
    output_ = L1Output{};
}

float L1Controller::lateral_accel_to_bank(float lateral_accel, float ground_speed) const {
    // bank = atan(lateral_accel / g)
    // For coordinated turn: L = m*g/cos(bank), centripetal = m*V^2/R
    // => tan(bank) = V^2 / (R*g) = a_lateral / g
    float bank = std::atan2(lateral_accel, GRAVITY);

    // Clamp to bank limit
    return math::constrain(bank, -config_.bank_limit_rad, config_.bank_limit_rad);
}

} // namespace fw_uav
