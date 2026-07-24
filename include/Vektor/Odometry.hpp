#pragma once
#include "Vektor/Pose.hpp"
#include "Vektor/Units.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <memory>

namespace Vektor {

class TrackingWheel {
public:
    TrackingWheel(std::shared_ptr<pros::Rotation> sensor, Length diameter, double gear_ratio, Length offset);

    void reset();
    Length get_distance();
    Length get_offset() const { return offset_; }

private:
    std::shared_ptr<pros::Rotation> sensor_;
    Length diameter_;
    double gear_ratio_;
    Length offset_;
};

class Odometry {
public:
    Odometry(std::shared_ptr<TrackingWheel> forward_wheel,
             std::shared_ptr<TrackingWheel> lateral_wheel,
             std::shared_ptr<pros::IMU> imu,
             std::shared_ptr<pros::MotorGroup> left_motors = nullptr,
             std::shared_ptr<pros::MotorGroup> right_motors = nullptr,
             Length track_width = Length::inches(12.0),
             Length drive_wheel_diameter = Length::inches(3.25),
             double drive_gear_ratio = 1.0);
    ~Odometry();

    Odometry(const Odometry&) = delete;
    Odometry& operator=(const Odometry&) = delete;

    void start();
    void stop();

    Pose get_pose(CoordinateFrame frame = CoordinateFrame::ABSOLUTE);
    void set_pose(const Pose& pose, CoordinateFrame frame = CoordinateFrame::ABSOLUTE);
    void set_starting_pose(const Pose& absolute_start_pose);
    Pose get_starting_pose() const;

private:
    void run_loop();

    std::shared_ptr<TrackingWheel> forward_wheel_;
    std::shared_ptr<TrackingWheel> lateral_wheel_;
    std::shared_ptr<pros::IMU> imu_;
    
    std::shared_ptr<pros::MotorGroup> left_motors_;
    std::shared_ptr<pros::MotorGroup> right_motors_;
    Length track_width_;
    Length drive_wheel_diameter_;
    double drive_gear_ratio_;

    std::unique_ptr<pros::Task> task_;
    mutable pros::Mutex mutex_;
    Pose pose_{0.0_in, 0.0_in, 0.0_rad};          // pose stored in absolute field coordinates
    Pose starting_pose_{0.0_in, 0.0_in, 0.0_rad}; // absolute starting pose offset
    bool running_{false};
};

} // namespace Vektor
