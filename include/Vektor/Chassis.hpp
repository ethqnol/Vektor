#pragma once
#include "Vektor/Units.hpp"
#include "Vektor/Pose.hpp"
#include "Vektor/Odometry.hpp"
#include "Vektor/Action.hpp"
#include "Vektor/PID.hpp"
#include "Vektor/Feedforward.hpp"
#include "pros/motor_group.hpp"
#include <memory>
#include <vector>
#include <functional>

namespace Vektor {

struct DrivetrainConfig {
    Length wheel_diameter;
    Length track_width;
    double gear_ratio; // output teeth / input teeth
};

class DriveCurve {
public:
    explicit DriveCurve(double scale = 0.5, double deadband = 5.0) noexcept
        : scale_(scale), deadband_(deadband) {}

    double calculate(double input) const noexcept {
        if (std::abs(input) < deadband_) return 0.0;
        double norm = input / 127.0;
        double curved = norm * ((1.0 - scale_) + scale_ * norm * norm);
        return curved * 12.0;
    }

private:
    double scale_;
    double deadband_;
};

class Chassis {
public:
    class SequenceBuilder {
    public:
        explicit SequenceBuilder(Chassis& chassis);

        SequenceBuilder& move_to(Pose target, Time timeout, CoordinateFrame frame = CoordinateFrame::ABSOLUTE);
        SequenceBuilder& turn_to(Angle target, Time timeout);
        SequenceBuilder& move_to_pose(Pose target, Time timeout, double lead = 0.6, CoordinateFrame frame = CoordinateFrame::ABSOLUTE);
        SequenceBuilder& follow_path(std::vector<Pose> path, Length lookahead, Time timeout, CoordinateFrame frame = CoordinateFrame::ABSOLUTE);
        SequenceBuilder& move_relative(Length distance, Time timeout);
        SequenceBuilder& turn_relative(Angle delta_angle, Time timeout);
        SequenceBuilder& then(std::function<void()> callback);
        SequenceBuilder& wait(Time duration);
        SequenceBuilder& add_spatial_trigger(Pose point, Length radius, std::function<void()> callback, CoordinateFrame frame = CoordinateFrame::ABSOLUTE);

        void execute();

    private:
        Chassis& chassis_;
        std::vector<std::unique_ptr<Action>> actions_;
    };

    Chassis(std::shared_ptr<pros::MotorGroup> left_motors,
            std::shared_ptr<pros::MotorGroup> right_motors,
            std::shared_ptr<Odometry> odom,
            DrivetrainConfig config,
            PID::Gains lateral_gains,
            PID::Gains angular_gains,
            Feedforward::Constants feedforward_consts);

    SequenceBuilder new_sequence();

    std::shared_ptr<Odometry> get_odometry() const { return odom_; }
    std::shared_ptr<pros::MotorGroup> get_left_motors() const { return left_motors_; }
    std::shared_ptr<pros::MotorGroup> get_right_motors() const { return right_motors_; }
    DrivetrainConfig get_config() const { return config_; }
    PID::Gains get_lateral_gains() const { return lateral_gains_; }
    PID::Gains get_angular_gains() const { return angular_gains_; }
    Feedforward::Constants get_feedforward_constants() const { return feedforward_consts_; }

    void set_starting_pose(const Pose& absolute_start_pose);
    Pose get_pose(CoordinateFrame frame = CoordinateFrame::ABSOLUTE) const;
    void set_pose(const Pose& pose, CoordinateFrame frame = CoordinateFrame::ABSOLUTE);

    // Driver control (opcontrol) steering modes
    void arcade(double throttle, double turn, bool use_curves = true);
    void tank(double left_input, double right_input, bool use_curves = true);
    void curvature(double throttle, double turn, bool use_curves = true);
    void set_drive_curves(DriveCurve throttle_curve, DriveCurve turn_curve);

    void drive_voltage(double left_volts, double right_volts);
    void stop();

private:
    std::shared_ptr<pros::MotorGroup> left_motors_;
    std::shared_ptr<pros::MotorGroup> right_motors_;
    std::shared_ptr<Odometry> odom_;
    DrivetrainConfig config_;
    PID::Gains lateral_gains_;
    PID::Gains angular_gains_;
    Feedforward::Constants feedforward_consts_;

    DriveCurve throttle_curve_{0.5, 5.0};
    DriveCurve turn_curve_{0.5, 5.0};
};

class CallbackAction : public Action {
public:
    explicit CallbackAction(std::function<void()> cb) : cb_(cb) {}
    bool update(double) override {
        cb_();
        return true;
    }
private:
    std::function<void()> cb_;
};

class DelayAction : public Action {
public:
    explicit DelayAction(Time duration) : duration_(duration) {}
    void initialize() override { elapsed_ = 0.0; }
    bool update(double dt) override {
        elapsed_ += dt;
        return elapsed_ >= duration_.to_seconds();
    }
private:
    Time duration_;
    double elapsed_{0.0};
};

class TriggeredMovementAction : public Action {
public:
    TriggeredMovementAction(std::unique_ptr<Action> movement,
                            std::shared_ptr<Odometry> odom,
                            Pose trigger_pt,
                            Length radius,
                            std::function<void()> callback,
                            CoordinateFrame frame = CoordinateFrame::ABSOLUTE)
        : movement_(std::move(movement)),
          odom_(odom),
          trigger_pt_(trigger_pt),
          radius_(radius),
          callback_(callback),
          frame_(frame) {}

    void initialize() override {
        movement_->initialize();
        fired_ = false;
    }

    bool update(double dt) override {
        if (!fired_) {
            if (odom_->get_pose(frame_).distance_to(trigger_pt_) <= radius_) {
                callback_();
                fired_ = true;
            }
        }
        return movement_->update(dt);
    }

    void end(bool interrupted) override {
        movement_->end(interrupted);
    }

    double get_exit_velocity() const override {
        return movement_->get_exit_velocity();
    }

private:
    std::unique_ptr<Action> movement_;
    std::shared_ptr<Odometry> odom_;
    Pose trigger_pt_;
    Length radius_;
    std::function<void()> callback_;
    CoordinateFrame frame_;
    bool fired_{false};
};

} // namespace Vektor
