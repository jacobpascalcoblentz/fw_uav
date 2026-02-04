#include <gtest/gtest.h>
#include "fw_uav/core/types.h"

using namespace fw_uav;

// Vec3f tests
TEST(Vec3fTest, DefaultConstruction) {
    Vec3f v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3fTest, ValueConstruction) {
    Vec3f v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3fTest, Addition) {
    Vec3f a(1.0f, 2.0f, 3.0f);
    Vec3f b(4.0f, 5.0f, 6.0f);
    Vec3f c = a + b;
    EXPECT_FLOAT_EQ(c.x, 5.0f);
    EXPECT_FLOAT_EQ(c.y, 7.0f);
    EXPECT_FLOAT_EQ(c.z, 9.0f);
}

TEST(Vec3fTest, Subtraction) {
    Vec3f a(4.0f, 5.0f, 6.0f);
    Vec3f b(1.0f, 2.0f, 3.0f);
    Vec3f c = a - b;
    EXPECT_FLOAT_EQ(c.x, 3.0f);
    EXPECT_FLOAT_EQ(c.y, 3.0f);
    EXPECT_FLOAT_EQ(c.z, 3.0f);
}

TEST(Vec3fTest, ScalarMultiplication) {
    Vec3f v(1.0f, 2.0f, 3.0f);
    Vec3f c = v * 2.0f;
    EXPECT_FLOAT_EQ(c.x, 2.0f);
    EXPECT_FLOAT_EQ(c.y, 4.0f);
    EXPECT_FLOAT_EQ(c.z, 6.0f);
}

TEST(Vec3fTest, DotProduct) {
    Vec3f a(1.0f, 2.0f, 3.0f);
    Vec3f b(4.0f, 5.0f, 6.0f);
    float dot = a.dot(b);
    EXPECT_FLOAT_EQ(dot, 32.0f);  // 1*4 + 2*5 + 3*6 = 32
}

TEST(Vec3fTest, CrossProduct) {
    Vec3f a = Vec3f::unit_x();
    Vec3f b = Vec3f::unit_y();
    Vec3f c = a.cross(b);
    EXPECT_FLOAT_EQ(c.x, 0.0f);
    EXPECT_FLOAT_EQ(c.y, 0.0f);
    EXPECT_FLOAT_EQ(c.z, 1.0f);
}

TEST(Vec3fTest, Norm) {
    Vec3f v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.norm(), 5.0f);
}

TEST(Vec3fTest, Normalized) {
    Vec3f v(3.0f, 4.0f, 0.0f);
    Vec3f n = v.normalized();
    EXPECT_NEAR(n.norm(), 1.0f, 1e-6f);
    EXPECT_FLOAT_EQ(n.x, 0.6f);
    EXPECT_FLOAT_EQ(n.y, 0.8f);
}

// EulerAngles tests
TEST(EulerAnglesTest, DefaultConstruction) {
    EulerAngles e;
    EXPECT_FLOAT_EQ(e.roll, 0.0f);
    EXPECT_FLOAT_EQ(e.pitch, 0.0f);
    EXPECT_FLOAT_EQ(e.yaw, 0.0f);
}

TEST(EulerAnglesTest, DegreesConversion) {
    EulerAngles e = EulerAngles::from_degrees(90.0f, 45.0f, 180.0f);
    EulerAngles deg = e.to_degrees();
    EXPECT_NEAR(deg.roll, 90.0f, 1e-4f);
    EXPECT_NEAR(deg.pitch, 45.0f, 1e-4f);
    EXPECT_NEAR(deg.yaw, 180.0f, 1e-4f);
}

// Quaternion tests
TEST(QuaternionTest, DefaultIsIdentity) {
    Quaternion q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, IdentityRotatesNothing) {
    Quaternion q = Quaternion::identity();
    Vec3f v(1.0f, 2.0f, 3.0f);
    Vec3f rotated = q.rotate(v);
    EXPECT_NEAR(rotated.x, v.x, 1e-6f);
    EXPECT_NEAR(rotated.y, v.y, 1e-6f);
    EXPECT_NEAR(rotated.z, v.z, 1e-6f);
}

TEST(QuaternionTest, FromEulerRoundTrip) {
    EulerAngles original = EulerAngles::from_degrees(30.0f, 20.0f, 45.0f);
    Quaternion q = Quaternion::from_euler(original);
    EulerAngles recovered = q.to_euler();
    EulerAngles orig_deg = original.to_degrees();
    EulerAngles rec_deg = recovered.to_degrees();
    EXPECT_NEAR(rec_deg.roll, orig_deg.roll, 1e-4f);
    EXPECT_NEAR(rec_deg.pitch, orig_deg.pitch, 1e-4f);
    EXPECT_NEAR(rec_deg.yaw, orig_deg.yaw, 1e-4f);
}

TEST(QuaternionTest, Rotation90DegreesAboutZ) {
    Quaternion q = Quaternion::from_axis_angle(Vec3f::unit_z(), math::HALF_PI);
    Vec3f v = Vec3f::unit_x();
    Vec3f rotated = q.rotate(v);
    EXPECT_NEAR(rotated.x, 0.0f, 1e-6f);
    EXPECT_NEAR(rotated.y, 1.0f, 1e-6f);
    EXPECT_NEAR(rotated.z, 0.0f, 1e-6f);
}

// NEDPosition tests
TEST(NEDPositionTest, AltitudeAGL) {
    NEDPosition p(100.0f, 50.0f, -200.0f);  // 200m up
    EXPECT_FLOAT_EQ(p.altitude_agl(), 200.0f);
}

// Math utilities
TEST(MathTest, WrapPi) {
    EXPECT_NEAR(math::wrap_pi(0.0f), 0.0f, 1e-6f);
    EXPECT_NEAR(math::wrap_pi(math::PI), math::PI, 1e-6f);
    EXPECT_NEAR(math::wrap_pi(math::PI + 0.1f), -math::PI + 0.1f, 1e-5f);
    EXPECT_NEAR(math::wrap_pi(-math::PI - 0.1f), math::PI - 0.1f, 1e-5f);
}

TEST(MathTest, Constrain) {
    EXPECT_FLOAT_EQ(math::constrain(5.0f, 0.0f, 10.0f), 5.0f);
    EXPECT_FLOAT_EQ(math::constrain(-5.0f, 0.0f, 10.0f), 0.0f);
    EXPECT_FLOAT_EQ(math::constrain(15.0f, 0.0f, 10.0f), 10.0f);
}

TEST(MathTest, Lerp) {
    EXPECT_FLOAT_EQ(math::lerp(0.0f, 10.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(math::lerp(0.0f, 10.0f, 1.0f), 10.0f);
    EXPECT_FLOAT_EQ(math::lerp(0.0f, 10.0f, 0.5f), 5.0f);
}

TEST(MathTest, GeoDistance) {
    GeoPosition a(37.7749, -122.4194, 0.0f);  // San Francisco
    GeoPosition b(34.0522, -118.2437, 0.0f);  // Los Angeles
    double dist = math::geo_distance_m(a, b);
    EXPECT_NEAR(dist, 559000.0, 5000.0);  // ~559 km, allow 5km tolerance
}
