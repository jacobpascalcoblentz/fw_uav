#include <gtest/gtest.h>
#include "fw_uav/core/units.h"

using namespace fw_uav;
using namespace fw_uav::units;

// Unit wrapper tests
TEST(UnitTest, DefaultConstruction) {
    Meters m;
    EXPECT_FLOAT_EQ(m.value(), 0.0f);
}

TEST(UnitTest, ValueConstruction) {
    Meters m(100.0f);
    EXPECT_FLOAT_EQ(m.value(), 100.0f);
}

TEST(UnitTest, Arithmetic) {
    Meters a(10.0f);
    Meters b(3.0f);

    EXPECT_FLOAT_EQ((a + b).value(), 13.0f);
    EXPECT_FLOAT_EQ((a - b).value(), 7.0f);
    EXPECT_FLOAT_EQ((-a).value(), -10.0f);
    EXPECT_FLOAT_EQ((a * 2.0f).value(), 20.0f);
    EXPECT_FLOAT_EQ((2.0f * a).value(), 20.0f);
    EXPECT_FLOAT_EQ((a / 2.0f).value(), 5.0f);
}

TEST(UnitTest, Comparison) {
    Meters a(10.0f);
    Meters b(20.0f);
    Meters c(10.0f);

    EXPECT_TRUE(a == c);
    EXPECT_TRUE(a != b);
    EXPECT_TRUE(a < b);
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(a <= c);
    EXPECT_TRUE(b > a);
    EXPECT_TRUE(b >= a);
    EXPECT_TRUE(c >= a);
}

TEST(UnitTest, CompoundAssignment) {
    Meters m(10.0f);
    m += Meters(5.0f);
    EXPECT_FLOAT_EQ(m.value(), 15.0f);

    m -= Meters(3.0f);
    EXPECT_FLOAT_EQ(m.value(), 12.0f);

    m *= 2.0f;
    EXPECT_FLOAT_EQ(m.value(), 24.0f);

    m /= 4.0f;
    EXPECT_FLOAT_EQ(m.value(), 6.0f);
}

// Distance conversions
TEST(DistanceConversionTest, MetersToFeet) {
    Meters m(1.0f);
    Feet f = to_feet(m);
    EXPECT_NEAR(f.value(), 3.28084f, 0.001f);
}

TEST(DistanceConversionTest, FeetToMeters) {
    Feet f(3.28084f);
    Meters m = to_meters(f);
    EXPECT_NEAR(m.value(), 1.0f, 0.001f);
}

TEST(DistanceConversionTest, KilometersToMeters) {
    Kilometers km(1.0f);
    Meters m = to_meters(km);
    EXPECT_FLOAT_EQ(m.value(), 1000.0f);
}

TEST(DistanceConversionTest, NauticalMilesToMeters) {
    NauticalMiles nm(1.0f);
    Meters m = to_meters(nm);
    EXPECT_FLOAT_EQ(m.value(), 1852.0f);
}

// Speed conversions
TEST(SpeedConversionTest, KnotsToMps) {
    Knots kts(1.0f);
    MetersPerSecond mps = to_mps(kts);
    EXPECT_NEAR(mps.value(), 0.514444f, 0.0001f);
}

TEST(SpeedConversionTest, MpsToKnots) {
    MetersPerSecond mps(1.0f);
    Knots kts = to_knots(mps);
    EXPECT_NEAR(kts.value(), 1.94384f, 0.001f);
}

TEST(SpeedConversionTest, KphToMps) {
    KilometersPerHour kph(3.6f);
    MetersPerSecond mps = to_mps(kph);
    EXPECT_NEAR(mps.value(), 1.0f, 0.001f);
}

// Angle conversions
TEST(AngleConversionTest, DegreesToRadians) {
    Degrees d(180.0f);
    Radians r = to_radians(d);
    EXPECT_NEAR(r.value(), PI, 0.0001f);
}

TEST(AngleConversionTest, RadiansToDegrees) {
    Radians r(PI);
    Degrees d = to_degrees(r);
    EXPECT_NEAR(d.value(), 180.0f, 0.001f);
}

// Time conversions
TEST(TimeConversionTest, MillisecondsToSeconds) {
    Milliseconds ms(1000.0f);
    Seconds s = to_seconds(ms);
    EXPECT_FLOAT_EQ(s.value(), 1.0f);
}

TEST(TimeConversionTest, MicrosecondsToSeconds) {
    Microseconds us(1000000.0f);
    Seconds s = to_seconds(us);
    EXPECT_FLOAT_EQ(s.value(), 1.0f);
}

TEST(TimeConversionTest, SecondsToMs) {
    Seconds s(1.5f);
    Milliseconds ms = to_ms(s);
    EXPECT_FLOAT_EQ(ms.value(), 1500.0f);
}

// Pressure conversions
TEST(PressureConversionTest, HpaToPa) {
    Hectopascals hpa(1013.25f);
    Pascals pa = to_pascals(hpa);
    EXPECT_FLOAT_EQ(pa.value(), 101325.0f);
}

TEST(PressureConversionTest, MbarToPa) {
    Millibars mb(1013.25f);
    Pascals pa = to_pascals(mb);
    EXPECT_FLOAT_EQ(pa.value(), 101325.0f);
}

// Temperature conversions
TEST(TemperatureConversionTest, FahrenheitToCelsius) {
    Fahrenheit f(32.0f);
    Celsius c = to_celsius(f);
    EXPECT_NEAR(c.value(), 0.0f, 0.001f);

    Fahrenheit f2(212.0f);
    Celsius c2 = to_celsius(f2);
    EXPECT_NEAR(c2.value(), 100.0f, 0.001f);
}

TEST(TemperatureConversionTest, CelsiusToKelvin) {
    Celsius c(0.0f);
    Kelvin k = to_kelvin(c);
    EXPECT_NEAR(k.value(), 273.15f, 0.001f);
}

TEST(TemperatureConversionTest, KelvinToCelsius) {
    Kelvin k(273.15f);
    Celsius c = to_celsius(k);
    EXPECT_NEAR(c.value(), 0.0f, 0.001f);
}

// User-defined literals
TEST(LiteralsTest, Distance) {
    using namespace literals;

    auto m = 100.0_m;
    EXPECT_FLOAT_EQ(m.value(), 100.0f);

    auto ft = 100_ft;
    EXPECT_FLOAT_EQ(ft.value(), 100.0f);

    auto km = 1.5_km;
    EXPECT_FLOAT_EQ(km.value(), 1.5f);
}

TEST(LiteralsTest, Speed) {
    using namespace literals;

    auto mps = 10.0_mps;
    EXPECT_FLOAT_EQ(mps.value(), 10.0f);

    auto kts = 100_kts;
    EXPECT_FLOAT_EQ(kts.value(), 100.0f);
}

TEST(LiteralsTest, Angle) {
    using namespace literals;

    auto rad = 1.57_rad;
    EXPECT_NEAR(rad.value(), 1.57f, 0.001f);

    auto deg = 90_deg;
    EXPECT_FLOAT_EQ(deg.value(), 90.0f);
}

TEST(LiteralsTest, Time) {
    using namespace literals;

    auto s = 1.5_s;
    EXPECT_FLOAT_EQ(s.value(), 1.5f);

    auto ms = 500_ms;
    EXPECT_FLOAT_EQ(ms.value(), 500.0f);

    auto us = 1000_us;
    EXPECT_FLOAT_EQ(us.value(), 1000.0f);
}

// Atmosphere functions
TEST(AtmosphereTest, PressureAltitude) {
    // At sea level pressure, altitude should be 0
    Meters alt = atm::pressure_altitude(Pascals(101325.0f));
    EXPECT_NEAR(alt.value(), 0.0f, 1.0f);

    // At ~900 hPa, should be around 1000m
    Meters alt1000 = atm::pressure_altitude(Pascals(89874.6f));
    EXPECT_NEAR(alt1000.value(), 1000.0f, 50.0f);
}

TEST(AtmosphereTest, AirDensity) {
    // At sea level standard conditions
    float rho = atm::air_density(Pascals(101325.0f), Celsius(15.0f));
    EXPECT_NEAR(rho, 1.225f, 0.01f);
}

TEST(AtmosphereTest, IndicatedAirspeed) {
    // Zero differential pressure = zero airspeed
    MetersPerSecond zero = atm::indicated_airspeed(Pascals(0.0f));
    EXPECT_FLOAT_EQ(zero.value(), 0.0f);

    // Negative differential pressure = zero airspeed
    MetersPerSecond neg = atm::indicated_airspeed(Pascals(-10.0f));
    EXPECT_FLOAT_EQ(neg.value(), 0.0f);

    // Typical pitot pressure at ~50 m/s
    // dp = 0.5 * rho * v^2 = 0.5 * 1.225 * 50^2 = 1531.25 Pa
    MetersPerSecond ias = atm::indicated_airspeed(Pascals(1531.25f));
    EXPECT_NEAR(ias.value(), 50.0f, 0.5f);
}
