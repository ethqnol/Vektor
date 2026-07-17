#pragma once
#include "Vektor/Units.hpp"
#include <cmath>

namespace Vektor {

enum class CoordinateFrame {
    ABSOLUTE, // absolute field coordinates (center of field is 0,0 facing 0 deg North)
    RELATIVE  // relative coordinates (robot starting position is 0,0 facing 0 deg)
};

struct Pose {
    Length x{0.0_in};
    Length y{0.0_in};
    Angle theta{0.0_rad};

    constexpr Pose() noexcept = default;
    constexpr Pose(Length x_val, Length y_val, Angle theta_val) noexcept
        : x(x_val), y(y_val), theta(theta_val) {}

    Length distance_to(const Pose& other) const noexcept {
        double dx = (other.x - x).to_inches();
        double dy = (other.y - y).to_inches();
        return Length::inches(std::sqrt(dx * dx + dy * dy));
    }

    Angle angle_to(const Pose& other) const noexcept {
        double dx = (other.x - x).to_inches();
        double dy = (other.y - y).to_inches();
        return Angle::rad(std::atan2(dy, dx));
    }

    // Converts a relative pose (where start is 0,0,0) into absolute field coordinates
    Pose to_absolute(const Pose& starting_pose) const noexcept {
        double s_th = starting_pose.theta.to_rad();
        double cos_s = std::cos(s_th);
        double sin_s = std::sin(s_th);

        double rx = x.to_inches();
        double ry = y.to_inches();

        double abs_x = starting_pose.x.to_inches() + (rx * cos_s - ry * sin_s);
        double abs_y = starting_pose.y.to_inches() + (rx * sin_s + ry * cos_s);
        Angle abs_th = (starting_pose.theta + theta).constrain();

        return Pose(Length::inches(abs_x), Length::inches(abs_y), abs_th);
    }

    // Converts an absolute field pose into relative coordinates (where start is 0,0,0)
    Pose to_relative(const Pose& starting_pose) const noexcept {
        double s_th = starting_pose.theta.to_rad();
        double cos_s = std::cos(s_th);
        double sin_s = std::sin(s_th);

        double dx = (x - starting_pose.x).to_inches();
        double dy = (y - starting_pose.y).to_inches();

        double rel_x = dx * cos_s + dy * sin_s;
        double rel_y = -dx * sin_s + dy * cos_s;
        Angle rel_th = (theta - starting_pose.theta).constrain();

        return Pose(Length::inches(rel_x), Length::inches(rel_y), rel_th);
    }
};

} // namespace Vektor
