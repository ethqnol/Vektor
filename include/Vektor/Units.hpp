#pragma once
#include <cmath>

namespace Vektor {

class Length {
public:
    constexpr Length() noexcept : inches_(0.0) {}
    explicit constexpr Length(double inches) noexcept : inches_(inches) {}

    static constexpr Length inches(double val) noexcept { return Length(val); }
    static constexpr Length meters(double val) noexcept { return Length(val * 39.37007874); }
    static constexpr Length cm(double val) noexcept { return Length(val * 0.3937007874); }

    constexpr double to_inches() const noexcept { return inches_; }
    constexpr double to_meters() const noexcept { return inches_ * 0.0254; }
    constexpr double to_cm() const noexcept { return inches_ * 2.54; }

    constexpr Length operator+(Length other) const noexcept { return Length(inches_ + other.inches_); }
    constexpr Length operator-(Length other) const noexcept { return Length(inches_ - other.inches_); }
    constexpr Length operator-() const noexcept { return Length(-inches_); }
    constexpr Length operator*(double scalar) const noexcept { return Length(inches_ * scalar); }
    constexpr Length operator/(double scalar) const noexcept { return Length(inches_ / scalar); }
    constexpr double operator/(Length other) const noexcept { return inches_ / other.inches_; }

    constexpr Length& operator+=(Length other) noexcept { inches_ += other.inches_; return *this; }
    constexpr Length& operator-=(Length other) noexcept { inches_ -= other.inches_; return *this; }
    constexpr Length& operator*=(double scalar) noexcept { inches_ *= scalar; return *this; }
    constexpr Length& operator/=(double scalar) noexcept { inches_ /= scalar; return *this; }

    constexpr bool operator==(Length other) const noexcept { return inches_ == other.inches_; }
    constexpr bool operator!=(Length other) const noexcept { return inches_ != other.inches_; }
    constexpr bool operator<(Length other) const noexcept { return inches_ < other.inches_; }
    constexpr bool operator<=(Length other) const noexcept { return inches_ <= other.inches_; }
    constexpr bool operator>(Length other) const noexcept { return inches_ > other.inches_; }
    constexpr bool operator>=(Length other) const noexcept { return inches_ >= other.inches_; }

private:
    double inches_;
};

constexpr Length operator*(double scalar, Length length) noexcept { return Length::inches(length.to_inches() * scalar); }

class Angle {
public:
    constexpr Angle() noexcept : rad_(0.0) {}
    explicit constexpr Angle(double rad) noexcept : rad_(rad) {}

    static constexpr Angle rad(double val) noexcept { return Angle(val); }
    static constexpr Angle deg(double val) noexcept { return Angle(val * M_PI / 180.0); }

    constexpr double to_rad() const noexcept { return rad_; }
    constexpr double to_deg() const noexcept { return rad_ * 180.0 / M_PI; }

    Angle constrain() const noexcept {
        double r = std::fmod(rad_, 2.0 * M_PI);
        if (r > M_PI) r -= 2.0 * M_PI;
        if (r <= -M_PI) r += 2.0 * M_PI;
        return Angle(r);
    }

    constexpr Angle operator+(Angle other) const noexcept { return Angle(rad_ + other.rad_); }
    constexpr Angle operator-(Angle other) const noexcept { return Angle(rad_ - other.rad_); }
    constexpr Angle operator-() const noexcept { return Angle(-rad_); }
    constexpr Angle operator*(double scalar) const noexcept { return Angle(rad_ * scalar); }
    constexpr Angle operator/(double scalar) const noexcept { return Angle(rad_ / scalar); }
    constexpr double operator/(Angle other) const noexcept { return rad_ / other.rad_; }

    constexpr Angle& operator+=(Angle other) noexcept { rad_ += other.rad_; return *this; }
    constexpr Angle& operator-=(Angle other) noexcept { rad_ -= other.rad_; return *this; }
    constexpr Angle& operator*=(double scalar) noexcept { rad_ *= scalar; return *this; }
    constexpr Angle& operator/=(double scalar) noexcept { rad_ /= scalar; return *this; }

    constexpr bool operator==(Angle other) const noexcept { return rad_ == other.rad_; }
    constexpr bool operator!=(Angle other) const noexcept { return rad_ != other.rad_; }
    constexpr bool operator<(Angle other) const noexcept { return rad_ < other.rad_; }
    constexpr bool operator<=(Angle other) const noexcept { return rad_ <= other.rad_; }
    constexpr bool operator>(Angle other) const noexcept { return rad_ > other.rad_; }
    constexpr bool operator>=(Angle other) const noexcept { return rad_ >= other.rad_; }

private:
    double rad_;
};

constexpr Angle operator*(double scalar, Angle angle) noexcept { return Angle::rad(angle.to_rad() * scalar); }

class Time {
public:
    constexpr Time() noexcept : seconds_(0.0) {}
    explicit constexpr Time(double seconds) noexcept : seconds_(seconds) {}

    static constexpr Time seconds(double val) noexcept { return Time(val); }
    static constexpr Time millis(double val) noexcept { return Time(val / 1000.0); }

    constexpr double to_seconds() const noexcept { return seconds_; }
    constexpr double to_millis() const noexcept { return seconds_ * 1000.0; }

    constexpr Time operator+(Time other) const noexcept { return Time(seconds_ + other.seconds_); }
    constexpr Time operator-(Time other) const noexcept { return Time(seconds_ - other.seconds_); }
    constexpr Time operator-() const noexcept { return Time(-seconds_); }
    constexpr Time operator*(double scalar) const noexcept { return Time(seconds_ * scalar); }
    constexpr Time operator/(double scalar) const noexcept { return Time(seconds_ / scalar); }
    constexpr double operator/(Time other) const noexcept { return seconds_ / other.seconds_; }

    constexpr Time& operator+=(Time other) noexcept { seconds_ += other.seconds_; return *this; }
    constexpr Time& operator-=(Time other) noexcept { seconds_ -= other.seconds_; return *this; }
    constexpr Time& operator*=(double scalar) noexcept { seconds_ *= scalar; return *this; }
    constexpr Time& operator/=(double scalar) noexcept { seconds_ /= scalar; return *this; }

    constexpr bool operator==(Time other) const noexcept { return seconds_ == other.seconds_; }
    constexpr bool operator!=(Time other) const noexcept { return seconds_ != other.seconds_; }
    constexpr bool operator<(Time other) const noexcept { return seconds_ < other.seconds_; }
    constexpr bool operator<=(Time other) const noexcept { return seconds_ <= other.seconds_; }
    constexpr bool operator>(Time other) const noexcept { return seconds_ > other.seconds_; }
    constexpr bool operator>=(Time other) const noexcept { return seconds_ >= other.seconds_; }

private:
    double seconds_;
};

constexpr Time operator*(double scalar, Time time) noexcept { return Time::seconds(time.to_seconds() * scalar); }

constexpr Length operator""_in(long double val) noexcept { return Length::inches(static_cast<double>(val)); }
constexpr Length operator""_in(unsigned long long val) noexcept { return Length::inches(static_cast<double>(val)); }
constexpr Length operator""_m(long double val) noexcept { return Length::meters(static_cast<double>(val)); }
constexpr Length operator""_m(unsigned long long val) noexcept { return Length::meters(static_cast<double>(val)); }
constexpr Length operator""_cm(long double val) noexcept { return Length::cm(static_cast<double>(val)); }
constexpr Length operator""_cm(unsigned long long val) noexcept { return Length::cm(static_cast<double>(val)); }

constexpr Angle operator""_rad(long double val) noexcept { return Angle::rad(static_cast<double>(val)); }
constexpr Angle operator""_rad(unsigned long long val) noexcept { return Angle::rad(static_cast<double>(val)); }
constexpr Angle operator""_deg(long double val) noexcept { return Angle::deg(static_cast<double>(val)); }
constexpr Angle operator""_deg(unsigned long long val) noexcept { return Angle::deg(static_cast<double>(val)); }

constexpr Time operator""_sec(long double val) noexcept { return Time::seconds(static_cast<double>(val)); }
constexpr Time operator""_sec(unsigned long long val) noexcept { return Time::seconds(static_cast<double>(val)); }
constexpr Time operator""_ms(long double val) noexcept { return Time::millis(static_cast<double>(val)); }
constexpr Time operator""_ms(unsigned long long val) noexcept { return Time::millis(static_cast<double>(val)); }

} // namespace Vektor
