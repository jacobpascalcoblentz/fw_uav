#pragma once

#include <cmath>

namespace fw_uav {
namespace units {

// Type-safe unit wrapper template
// Prevents accidental mixing of units (e.g., meters vs feet)
template<typename Tag, typename T = float>
class Unit {
public:
    constexpr Unit() : value_(T{0}) {}
    constexpr explicit Unit(T value) : value_(value) {}

    constexpr T value() const { return value_; }
    constexpr explicit operator T() const { return value_; }

    // Arithmetic with same units
    constexpr Unit operator+(Unit other) const { return Unit(value_ + other.value_); }
    constexpr Unit operator-(Unit other) const { return Unit(value_ - other.value_); }
    constexpr Unit operator-() const { return Unit(-value_); }

    // Scalar multiplication/division
    constexpr Unit operator*(T scalar) const { return Unit(value_ * scalar); }
    constexpr Unit operator/(T scalar) const { return Unit(value_ / scalar); }

    // Comparison
    constexpr bool operator==(Unit other) const { return value_ == other.value_; }
    constexpr bool operator!=(Unit other) const { return value_ != other.value_; }
    constexpr bool operator<(Unit other) const { return value_ < other.value_; }
    constexpr bool operator<=(Unit other) const { return value_ <= other.value_; }
    constexpr bool operator>(Unit other) const { return value_ > other.value_; }
    constexpr bool operator>=(Unit other) const { return value_ >= other.value_; }

    // Compound assignment
    Unit& operator+=(Unit other) { value_ += other.value_; return *this; }
    Unit& operator-=(Unit other) { value_ -= other.value_; return *this; }
    Unit& operator*=(T scalar) { value_ *= scalar; return *this; }
    Unit& operator/=(T scalar) { value_ /= scalar; return *this; }

    // Absolute value
    Unit abs() const { return Unit(std::fabs(value_)); }

private:
    T value_;
};

template<typename Tag, typename T>
constexpr Unit<Tag, T> operator*(T scalar, Unit<Tag, T> u) { return u * scalar; }

// Unit tags
struct MetersTag {};
struct FeetTag {};
struct KilometersTag {};
struct NauticalMilesTag {};

struct MetersPerSecondTag {};
struct KnotsTag {};
struct KilometersPerHourTag {};
struct FeetPerMinuteTag {};

struct RadiansTag {};
struct DegreesTag {};

struct RadiansPerSecondTag {};
struct DegreesPerSecondTag {};

struct SecondsTag {};
struct MillisecondsTag {};
struct MicrosecondsTag {};

struct PascalsTag {};
struct HectopascalsTag {};
struct MillibarsTag {};

struct CelsiusTag {};
struct FahrenheitTag {};
struct KelvinTag {};

// Distance units
using Meters = Unit<MetersTag>;
using Feet = Unit<FeetTag>;
using Kilometers = Unit<KilometersTag>;
using NauticalMiles = Unit<NauticalMilesTag>;

// Speed units
using MetersPerSecond = Unit<MetersPerSecondTag>;
using Knots = Unit<KnotsTag>;
using KilometersPerHour = Unit<KilometersPerHourTag>;
using FeetPerMinute = Unit<FeetPerMinuteTag>;

// Angle units
using Radians = Unit<RadiansTag>;
using Degrees = Unit<DegreesTag>;

// Angular rate units
using RadiansPerSecond = Unit<RadiansPerSecondTag>;
using DegreesPerSecond = Unit<DegreesPerSecondTag>;

// Time units
using Seconds = Unit<SecondsTag>;
using Milliseconds = Unit<MillisecondsTag>;
using Microseconds = Unit<MicrosecondsTag>;

// Pressure units
using Pascals = Unit<PascalsTag>;
using Hectopascals = Unit<HectopascalsTag>;
using Millibars = Unit<MillibarsTag>;

// Temperature units
using Celsius = Unit<CelsiusTag>;
using Fahrenheit = Unit<FahrenheitTag>;
using Kelvin = Unit<KelvinTag>;

// Conversion functions

// Distance conversions
constexpr float METERS_PER_FOOT = 0.3048f;
constexpr float METERS_PER_KILOMETER = 1000.0f;
constexpr float METERS_PER_NAUTICAL_MILE = 1852.0f;

constexpr Meters to_meters(Feet f) { return Meters(f.value() * METERS_PER_FOOT); }
constexpr Meters to_meters(Kilometers k) { return Meters(k.value() * METERS_PER_KILOMETER); }
constexpr Meters to_meters(NauticalMiles nm) { return Meters(nm.value() * METERS_PER_NAUTICAL_MILE); }
constexpr Meters to_meters(Meters m) { return m; }

constexpr Feet to_feet(Meters m) { return Feet(m.value() / METERS_PER_FOOT); }
constexpr Kilometers to_kilometers(Meters m) { return Kilometers(m.value() / METERS_PER_KILOMETER); }
constexpr NauticalMiles to_nautical_miles(Meters m) { return NauticalMiles(m.value() / METERS_PER_NAUTICAL_MILE); }

// Speed conversions
constexpr float MPS_PER_KNOT = 0.514444f;
constexpr float MPS_PER_KPH = 1.0f / 3.6f;
constexpr float MPS_PER_FPM = METERS_PER_FOOT / 60.0f;

constexpr MetersPerSecond to_mps(Knots kts) { return MetersPerSecond(kts.value() * MPS_PER_KNOT); }
constexpr MetersPerSecond to_mps(KilometersPerHour kph) { return MetersPerSecond(kph.value() * MPS_PER_KPH); }
constexpr MetersPerSecond to_mps(FeetPerMinute fpm) { return MetersPerSecond(fpm.value() * MPS_PER_FPM); }
constexpr MetersPerSecond to_mps(MetersPerSecond mps) { return mps; }

constexpr Knots to_knots(MetersPerSecond mps) { return Knots(mps.value() / MPS_PER_KNOT); }
constexpr KilometersPerHour to_kph(MetersPerSecond mps) { return KilometersPerHour(mps.value() / MPS_PER_KPH); }
constexpr FeetPerMinute to_fpm(MetersPerSecond mps) { return FeetPerMinute(mps.value() / MPS_PER_FPM); }

// Angle conversions
constexpr float PI = 3.14159265358979323846f;
constexpr float RAD_PER_DEG = PI / 180.0f;

constexpr Radians to_radians(Degrees d) { return Radians(d.value() * RAD_PER_DEG); }
constexpr Radians to_radians(Radians r) { return r; }
constexpr Degrees to_degrees(Radians r) { return Degrees(r.value() / RAD_PER_DEG); }
constexpr Degrees to_degrees(Degrees d) { return d; }

// Angular rate conversions
constexpr RadiansPerSecond to_radps(DegreesPerSecond dps) { return RadiansPerSecond(dps.value() * RAD_PER_DEG); }
constexpr RadiansPerSecond to_radps(RadiansPerSecond rps) { return rps; }
constexpr DegreesPerSecond to_degps(RadiansPerSecond rps) { return DegreesPerSecond(rps.value() / RAD_PER_DEG); }

// Time conversions
constexpr Seconds to_seconds(Milliseconds ms) { return Seconds(ms.value() / 1000.0f); }
constexpr Seconds to_seconds(Microseconds us) { return Seconds(us.value() / 1000000.0f); }
constexpr Seconds to_seconds(Seconds s) { return s; }

constexpr Milliseconds to_ms(Seconds s) { return Milliseconds(s.value() * 1000.0f); }
constexpr Microseconds to_us(Seconds s) { return Microseconds(s.value() * 1000000.0f); }

// Pressure conversions
constexpr float PA_PER_HPA = 100.0f;

constexpr Pascals to_pascals(Hectopascals hpa) { return Pascals(hpa.value() * PA_PER_HPA); }
constexpr Pascals to_pascals(Millibars mb) { return Pascals(mb.value() * PA_PER_HPA); } // 1 mbar = 1 hPa
constexpr Pascals to_pascals(Pascals pa) { return pa; }

constexpr Hectopascals to_hpa(Pascals pa) { return Hectopascals(pa.value() / PA_PER_HPA); }
constexpr Millibars to_mbar(Pascals pa) { return Millibars(pa.value() / PA_PER_HPA); }

// Temperature conversions
constexpr Celsius to_celsius(Fahrenheit f) { return Celsius((f.value() - 32.0f) * 5.0f / 9.0f); }
constexpr Celsius to_celsius(Kelvin k) { return Celsius(k.value() - 273.15f); }
constexpr Celsius to_celsius(Celsius c) { return c; }

constexpr Fahrenheit to_fahrenheit(Celsius c) { return Fahrenheit(c.value() * 9.0f / 5.0f + 32.0f); }
constexpr Kelvin to_kelvin(Celsius c) { return Kelvin(c.value() + 273.15f); }

// User-defined literals (requires C++14 or later)
namespace literals {

// Distance
constexpr Meters operator""_m(long double v) { return Meters(static_cast<float>(v)); }
constexpr Meters operator""_m(unsigned long long v) { return Meters(static_cast<float>(v)); }
constexpr Feet operator""_ft(long double v) { return Feet(static_cast<float>(v)); }
constexpr Feet operator""_ft(unsigned long long v) { return Feet(static_cast<float>(v)); }
constexpr Kilometers operator""_km(long double v) { return Kilometers(static_cast<float>(v)); }
constexpr NauticalMiles operator""_nm(long double v) { return NauticalMiles(static_cast<float>(v)); }

// Speed
constexpr MetersPerSecond operator""_mps(long double v) { return MetersPerSecond(static_cast<float>(v)); }
constexpr MetersPerSecond operator""_mps(unsigned long long v) { return MetersPerSecond(static_cast<float>(v)); }
constexpr Knots operator""_kts(long double v) { return Knots(static_cast<float>(v)); }
constexpr Knots operator""_kts(unsigned long long v) { return Knots(static_cast<float>(v)); }
constexpr KilometersPerHour operator""_kph(long double v) { return KilometersPerHour(static_cast<float>(v)); }
constexpr FeetPerMinute operator""_fpm(long double v) { return FeetPerMinute(static_cast<float>(v)); }

// Angle
constexpr Radians operator""_rad(long double v) { return Radians(static_cast<float>(v)); }
constexpr Radians operator""_rad(unsigned long long v) { return Radians(static_cast<float>(v)); }
constexpr Degrees operator""_deg(long double v) { return Degrees(static_cast<float>(v)); }
constexpr Degrees operator""_deg(unsigned long long v) { return Degrees(static_cast<float>(v)); }

// Angular rate
constexpr RadiansPerSecond operator""_radps(long double v) { return RadiansPerSecond(static_cast<float>(v)); }
constexpr DegreesPerSecond operator""_degps(long double v) { return DegreesPerSecond(static_cast<float>(v)); }

// Time
constexpr Seconds operator""_s(long double v) { return Seconds(static_cast<float>(v)); }
constexpr Seconds operator""_s(unsigned long long v) { return Seconds(static_cast<float>(v)); }
constexpr Milliseconds operator""_ms(long double v) { return Milliseconds(static_cast<float>(v)); }
constexpr Milliseconds operator""_ms(unsigned long long v) { return Milliseconds(static_cast<float>(v)); }
constexpr Microseconds operator""_us(long double v) { return Microseconds(static_cast<float>(v)); }
constexpr Microseconds operator""_us(unsigned long long v) { return Microseconds(static_cast<float>(v)); }

// Pressure
constexpr Pascals operator""_Pa(long double v) { return Pascals(static_cast<float>(v)); }
constexpr Hectopascals operator""_hPa(long double v) { return Hectopascals(static_cast<float>(v)); }
constexpr Millibars operator""_mbar(long double v) { return Millibars(static_cast<float>(v)); }

// Temperature
constexpr Celsius operator""_C(long double v) { return Celsius(static_cast<float>(v)); }
constexpr Fahrenheit operator""_F(long double v) { return Fahrenheit(static_cast<float>(v)); }
constexpr Kelvin operator""_K(long double v) { return Kelvin(static_cast<float>(v)); }

} // namespace literals

// Standard atmosphere constants
namespace atm {

constexpr Pascals SEA_LEVEL_PRESSURE = Pascals(101325.0f);
constexpr Celsius SEA_LEVEL_TEMP = Celsius(15.0f);
constexpr float LAPSE_RATE_K_PER_M = 0.0065f;  // Temperature lapse rate in troposphere
constexpr float GAS_CONSTANT = 287.05f;        // Specific gas constant for dry air (J/kg/K)
constexpr float GRAVITY = 9.80665f;

// Calculate pressure altitude from static pressure
inline Meters pressure_altitude(Pascals pressure) {
    // Using barometric formula: h = (T0/L) * (1 - (P/P0)^(R*L/(g*M)))
    // Simplified for troposphere
    float p_ratio = pressure.value() / SEA_LEVEL_PRESSURE.value();
    float exponent = GAS_CONSTANT * LAPSE_RATE_K_PER_M / GRAVITY;
    float altitude = (to_kelvin(SEA_LEVEL_TEMP).value() / LAPSE_RATE_K_PER_M) *
                     (1.0f - std::pow(p_ratio, exponent));
    return Meters(altitude);
}

// Calculate air density from pressure and temperature
inline float air_density(Pascals pressure, Celsius temperature) {
    return pressure.value() / (GAS_CONSTANT * to_kelvin(temperature).value());
}

// Calculate indicated airspeed from differential pressure
// IAS = sqrt(2 * dp / rho_0) where rho_0 is sea level density
inline MetersPerSecond indicated_airspeed(Pascals differential_pressure) {
    constexpr float SEA_LEVEL_DENSITY = 1.225f;  // kg/m^3
    float dp = differential_pressure.value();
    if (dp <= 0.0f) return MetersPerSecond(0.0f);
    return MetersPerSecond(std::sqrt(2.0f * dp / SEA_LEVEL_DENSITY));
}

// Calculate true airspeed from IAS and air density
inline MetersPerSecond true_airspeed(MetersPerSecond ias, float density) {
    constexpr float SEA_LEVEL_DENSITY = 1.225f;
    return MetersPerSecond(ias.value() * std::sqrt(SEA_LEVEL_DENSITY / density));
}

} // namespace atm

} // namespace units
} // namespace fw_uav
