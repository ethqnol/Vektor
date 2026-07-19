#include "Vektor/Odometry.hpp"
#include <cmath>

namespace Vektor {

TrackingWheel::TrackingWheel(std::shared_ptr<pros::Rotation> sensor, Length diameter, double gear_ratio, Length offset)
    : sensor_(sensor), diameter_(diameter), gear_ratio_(gear_ratio), offset_(offset) {
    reset();
}

void TrackingWheel::reset() {
    if (sensor_) {
        sensor_->reset_position();
    }
}

Length TrackingWheel::get_distance() {
    if (!sensor_) return Length::inches(0.0);

    double centideg = static_cast<double>(sensor_->get_position());
    double deg = centideg / 100.0;
    double rad = deg * M_PI / 180.0;
    double wheel_rad = rad * gear_ratio_;

    return Length::inches((diameter_.to_inches() / 2.0) * wheel_rad);
}

Odometry::Odometry(std::shared_ptr<TrackingWheel> forward_wheel,
                   std::shared_ptr<TrackingWheel> lateral_wheel,
                   std::shared_ptr<pros::IMU> imu,
                   std::shared_ptr<pros::MotorGroup> left_motors,
                   std::shared_ptr<pros::MotorGroup> right_motors,
                   Length track_width,
                   Length drive_wheel_diameter,
                   double drive_gear_ratio)
    : forward_wheel_(forward_wheel),
      lateral_wheel_(lateral_wheel),
      imu_(imu),
      left_motors_(left_motors),
      right_motors_(right_motors),
      track_width_(track_width),
      drive_wheel_diameter_(drive_wheel_diameter),
      drive_gear_ratio_(drive_gear_ratio),
      pose_(0.0_in, 0.0_in, 0.0_rad),
      starting_pose_(0.0_in, 0.0_in, 0.0_rad) {}

Odometry::~Odometry() {
    stop();
}

void Odometry::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;

    running_ = true;
    task_ = std::make_unique<pros::Task>([this]() {
        run_loop();
    }, "VektorOdometry");
}

void Odometry::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        running_ = false;
    }
    task_.reset();
}

void Odometry::set_starting_pose(const Pose& absolute_start_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    starting_pose_ = absolute_start_pose;
    // Set current absolute pose equal to starting pose
    pose_ = absolute_start_pose;
    if (forward_wheel_) forward_wheel_->reset();
    if (lateral_wheel_) lateral_wheel_->reset();
    if (left_motors_) left_motors_->tare_position();
    if (right_motors_) right_motors_->tare_position();
}

Pose Odometry::get_starting_pose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return starting_pose_;
}

Pose Odometry::get_pose(CoordinateFrame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame == CoordinateFrame::RELATIVE) {
        return pose_.to_relative(starting_pose_);
    }
    return pose_;
}

void Odometry::set_pose(const Pose& pose, CoordinateFrame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame == CoordinateFrame::RELATIVE) {
        pose_ = pose.to_absolute(starting_pose_);
    } else {
        pose_ = pose;
    }
    if (forward_wheel_) forward_wheel_->reset();
    if (lateral_wheel_) lateral_wheel_->reset();
    if (left_motors_) left_motors_->tare_position();
    if (right_motors_) right_motors_->tare_position();
}

static double get_motor_distance(std::shared_ptr<pros::MotorGroup> motors, Length diameter, double gear_ratio) {
    if (!motors) return 0.0;
    double deg = motors->get_position();
    double rad = deg * M_PI / 180.0;
    double wheel_rad = rad * gear_ratio;
    return (diameter.to_inches() / 2.0) * wheel_rad;
}

void Odometry::run_loop() {
    if (imu_) {
        while (imu_->is_calibrating()) {
            pros::delay(10);
        }
    }

    if (forward_wheel_) forward_wheel_->reset();
    if (lateral_wheel_) lateral_wheel_->reset();
    if (left_motors_) left_motors_->tare_position();
    if (right_motors_) right_motors_->tare_position();

    double last_forward = forward_wheel_ ? forward_wheel_->get_distance().to_inches() : 0.0;
    double last_lateral = lateral_wheel_ ? lateral_wheel_->get_distance().to_inches() : 0.0;

    double last_theta = starting_pose_.theta.to_rad();
    if (imu_) {
        double r = imu_->get_rotation();
        if (!std::isnan(r) && r != PROS_ERR_F) {
            last_theta = starting_pose_.theta.to_rad() - (r * M_PI / 180.0);
        }
    }

    double last_left = get_motor_distance(left_motors_, drive_wheel_diameter_, drive_gear_ratio_);
    double last_right = get_motor_distance(right_motors_, drive_wheel_diameter_, drive_gear_ratio_);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_) break;
        }

        double cur_forward = forward_wheel_ ? forward_wheel_->get_distance().to_inches() : 0.0;
        double cur_lateral = lateral_wheel_ ? lateral_wheel_->get_distance().to_inches() : 0.0;

        double d_forward = cur_forward - last_forward;
        double d_lateral = cur_lateral - last_lateral;

        last_forward = cur_forward;
        last_lateral = cur_lateral;

        double d_theta = 0.0;
        double cur_theta = last_theta;
        bool imu_valid = false;

        if (imu_) {
            double r = imu_->get_rotation();
            if (!std::isnan(r) && r != PROS_ERR_F) {
                cur_theta = starting_pose_.theta.to_rad() - (r * M_PI / 180.0);
                d_theta = cur_theta - last_theta;
                last_theta = cur_theta;
                imu_valid = true;
            }
        }

        // fallback to drivetrain encoders if IMU disconnects or returns NaN
        if (!imu_valid && left_motors_ && right_motors_) {
            double cur_left = get_motor_distance(left_motors_, drive_wheel_diameter_, drive_gear_ratio_);
            double cur_right = get_motor_distance(right_motors_, drive_wheel_diameter_, drive_gear_ratio_);

            double d_left = cur_left - last_left;
            double d_right = cur_right - last_right;

            last_left = cur_left;
            last_right = cur_right;

            d_theta = (d_left - d_right) / track_width_.to_inches();
            cur_theta = last_theta + d_theta;
            last_theta = cur_theta;
        }

        double d_f_offset = forward_wheel_ ? forward_wheel_->get_offset().to_inches() : 0.0;
        double d_l_offset = lateral_wheel_ ? lateral_wheel_->get_offset().to_inches() : 0.0;

        double local_x = 0.0;
        double local_y = 0.0;

        if (std::abs(d_theta) < 1e-5) {
            local_x = d_lateral;
            local_y = d_forward;
        } else {
            local_x = 2.0 * std::sin(d_theta / 2.0) * ((d_lateral / d_theta) - d_l_offset);
            local_y = 2.0 * std::sin(d_theta / 2.0) * ((d_forward / d_theta) - d_f_offset);
        }

        double theta_avg = last_theta - (d_theta / 2.0);
        double cos_avg = std::cos(theta_avg);
        double sin_avg = std::sin(theta_avg);

        double dX = local_y * cos_avg - local_x * sin_avg;
        double dY = local_y * sin_avg + local_x * cos_avg;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pose_.x += Length::inches(dX);
            pose_.y += Length::inches(dY);
            pose_.theta = Angle::rad(cur_theta).constrain();
        }

        pros::delay(2);
    }
}

} // namespace Vektor
