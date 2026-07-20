#include "Vektor/Chassis.hpp"
#include "Vektor/Executor.hpp"
#include "Vektor/Motions.hpp"
#include <algorithm>

namespace Vektor {

Chassis::SequenceBuilder::SequenceBuilder(Chassis& chassis) : chassis_(chassis) {}

SequenceBuilder& Chassis::SequenceBuilder::move_to(Pose target, Time timeout, CoordinateFrame frame) {
    Pose abs_target = (frame == CoordinateFrame::RELATIVE && chassis_.get_odometry())
                        ? target.to_absolute(chassis_.get_odometry()->get_starting_pose())
                        : target;
    actions_.push_back(std::make_unique<MoveAction>(chassis_, abs_target, timeout));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::turn_to(Angle target, Time timeout) {
    actions_.push_back(std::make_unique<TurnAction>(chassis_, target, timeout));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::move_to_pose(Pose target, Time timeout, double lead, CoordinateFrame frame) {
    Pose abs_target = (frame == CoordinateFrame::RELATIVE && chassis_.get_odometry())
                        ? target.to_absolute(chassis_.get_odometry()->get_starting_pose())
                        : target;
    actions_.push_back(std::make_unique<MoveToPoseAction>(chassis_, abs_target, timeout, lead));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::follow_path(std::vector<Pose> path, Length lookahead, Time timeout, CoordinateFrame frame) {
    std::vector<Pose> abs_path = path;
    if (frame == CoordinateFrame::RELATIVE && chassis_.get_odometry()) {
        Pose start_pose = chassis_.get_odometry()->get_starting_pose();
        for (auto& pt : abs_path) {
            pt = pt.to_absolute(start_pose);
        }
    }
    actions_.push_back(std::make_unique<FollowPathAction>(chassis_, abs_path, lookahead, timeout));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::move_relative(Length distance, Time timeout) {
    actions_.push_back(std::make_unique<MoveRelativeAction>(chassis_, distance, timeout));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::turn_relative(Angle delta_angle, Time timeout) {
    actions_.push_back(std::make_unique<TurnRelativeAction>(chassis_, delta_angle, timeout));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::then(std::function<void()> callback) {
    actions_.push_back(std::make_unique<CallbackAction>(callback));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::wait(Time duration) {
    actions_.push_back(std::make_unique<DelayAction>(duration));
    return *this;
}

SequenceBuilder& Chassis::SequenceBuilder::add_spatial_trigger(Pose point, Length radius, std::function<void()> callback, CoordinateFrame frame) {
    if (actions_.empty()) return *this;

    auto last_action = std::move(actions_.back());
    actions_.pop_back();

    actions_.push_back(std::make_unique<TriggeredMovementAction>(
        std::move(last_action),
        chassis_.get_odometry(),
        point,
        radius,
        callback,
        frame
    ));
    return *this;
}

void Chassis::SequenceBuilder::execute() {
    Executor::get_instance().queue_action(std::make_unique<SequentialAction>(std::move(actions_)));
}

Chassis::Chassis(std::shared_ptr<pros::MotorGroup> left_motors,
                 std::shared_ptr<pros::MotorGroup> right_motors,
                 std::shared_ptr<Odometry> odom,
                 DrivetrainConfig config,
                 PID::Gains lateral_gains,
                 PID::Gains angular_gains,
                 Feedforward::Constants feedforward_consts)
    : left_motors_(left_motors),
      right_motors_(right_motors),
      odom_(odom),
      config_(config),
      lateral_gains_(lateral_gains),
      angular_gains_(angular_gains),
      feedforward_consts_(feedforward_consts) {}

Chassis::SequenceBuilder Chassis::new_sequence() {
    return SequenceBuilder(*this);
}

void Chassis::set_starting_pose(const Pose& absolute_start_pose) {
    if (odom_) {
        odom_->set_starting_pose(absolute_start_pose);
    }
}

Pose Chassis::get_pose(CoordinateFrame frame) const {
    if (odom_) {
        return odom_->get_pose(frame);
    }
    return Pose(0_in, 0_in, 0_deg);
}

void Chassis::set_pose(const Pose& pose, CoordinateFrame frame) {
    if (odom_) {
        odom_->set_pose(pose, frame);
    }
}

void Chassis::set_drive_curves(DriveCurve throttle_curve, DriveCurve turn_curve) {
    throttle_curve_ = throttle_curve;
    turn_curve_ = turn_curve;
}

void Chassis::arcade(double throttle, double turn, bool use_curves) {
    double t_volts = use_curves ? throttle_curve_.calculate(throttle) : (throttle / 127.0 * 12.0);
    double r_volts = use_curves ? turn_curve_.calculate(turn) : (turn / 127.0 * 12.0);

    double left = t_volts + r_volts;
    double right = t_volts - r_volts;

    double max_mag = std::max(std::abs(left), std::abs(right));
    if (max_mag > 12.0) {
        left = (left / max_mag) * 12.0;
        right = (right / max_mag) * 12.0;
    }

    drive_voltage(left, right);
}

void Chassis::tank(double left_input, double right_input, bool use_curves) {
    double left = use_curves ? throttle_curve_.calculate(left_input) : (left_input / 127.0 * 12.0);
    double right = use_curves ? throttle_curve_.calculate(right_input) : (right_input / 127.0 * 12.0);
    drive_voltage(left, right);
}

void Chassis::curvature(double throttle, double turn, bool use_curves) {
    if (std::abs(throttle) < 5.0) {
        arcade(throttle, turn, use_curves);
        return;
    }
    double t_volts = use_curves ? throttle_curve_.calculate(throttle) : (throttle / 127.0 * 12.0);
    double r_volts = use_curves ? turn_curve_.calculate(turn) : (turn / 127.0 * 12.0);

    double left = t_volts + std::abs(t_volts) * (r_volts / 12.0);
    double right = t_volts - std::abs(t_volts) * (r_volts / 12.0);

    drive_voltage(left, right);
}

void Chassis::drive_voltage(double left_volts, double right_volts) {
    if (left_motors_ && right_motors_) {
        left_motors_->move_voltage(static_cast<std::int32_t>(left_volts * 1000.0));
        right_motors_->move_voltage(static_cast<std::int32_t>(right_volts * 1000.0));
    }
}

void Chassis::stop() {
    if (left_motors_ && right_motors_) {
        left_motors_->brake();
        right_motors_->brake();
    }
}

} // namespace Vektor
