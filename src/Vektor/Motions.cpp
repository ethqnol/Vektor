#include "Vektor/Motions.hpp"
#include "Vektor/Chassis.hpp"
#include <algorithm>

namespace Vektor {

MoveAction::MoveAction(Chassis& chassis, Pose target, Time timeout)
    : chassis_(chassis),
      target_(target),
      timeout_(timeout),
      lateral_pid_(chassis.get_lateral_gains()),
      angular_pid_(chassis.get_angular_gains()),
      ff_(chassis.get_feedforward_constants()),
      lateral_exit_(0.5, 0.1),
      angular_exit_(1.5 * M_PI / 180.0, 0.1) {}

void MoveAction::initialize() {
    elapsed_time_ = 0.0;
    lateral_pid_.reset();
    angular_pid_.reset();
    lateral_exit_.reset();
    angular_exit_.reset();
}

bool MoveAction::update(double dt) {
    Pose current_pose = chassis_.get_odometry()->get_pose();

    double distance = current_pose.distance_to(target_).to_inches();
    double angle_to_target = current_pose.angle_to(target_).to_rad();
    double heading_err = angle_to_target - current_pose.theta.to_rad();

    Angle error_angle = Angle::rad(heading_err).constrain();
    double lateral_error = distance * std::cos(error_angle.to_rad());

    // disable steering corrections when close to point to stop atan2 noise oscillation
    double angular_error = 0.0;
    if (distance > 0.75) {
        angular_error = error_angle.to_rad();
    }

    double lateral_power = lateral_pid_.update(lateral_error, dt);
    double angular_power = angular_pid_.update(angular_error, dt);

    // suppress kS friction offset inside 0.25" band to stop motor hunting
    double lateral_ff = 0.0;
    if (std::abs(lateral_error) > 0.25) {
        lateral_ff = (lateral_error > 0) ? chassis_.get_feedforward_constants().kS : -chassis_.get_feedforward_constants().kS;
    }
    double lateral_volts = lateral_power + lateral_ff;

    double left_volts = std::clamp(lateral_volts - angular_power, -12.0, 12.0);
    double right_volts = std::clamp(lateral_volts + angular_power, -12.0, 12.0);

    chassis_.drive_voltage(left_volts, right_volts);

    bool settled = lateral_exit_.update(lateral_error, dt) && angular_exit_.update(angular_error, dt);
    elapsed_time_ += dt;
    return settled || (elapsed_time_ >= timeout_.to_seconds());
}

void MoveAction::end(bool) {
    chassis_.stop();
}

TurnAction::TurnAction(Chassis& chassis, Angle target_heading, Time timeout)
    : chassis_(chassis),
      target_heading_(target_heading),
      timeout_(timeout),
      pid_(chassis.get_angular_gains()),
      exit_cond_(1.0 * M_PI / 180.0, 0.1) {}

void TurnAction::initialize() {
    elapsed_time_ = 0.0;
    pid_.reset();
    exit_cond_.reset();
}

bool TurnAction::update(double dt) {
    Pose current_pose = chassis_.get_odometry()->get_pose();

    Angle angular_error = (target_heading_ - current_pose.theta).constrain();
    double error_rad = angular_error.to_rad();

    double angular_power = pid_.update(error_rad, dt);

    // suppress kS friction offset inside 0.5 deg band to stop motor whine
    double turn_ff = 0.0;
    if (std::abs(error_rad) > (0.5 * M_PI / 180.0)) {
        turn_ff = (error_rad > 0) ? chassis_.get_feedforward_constants().kS : -chassis_.get_feedforward_constants().kS;
    }
    double angular_volts = angular_power + turn_ff;

    double left_volts = std::clamp(-angular_volts, -12.0, 12.0);
    double right_volts = std::clamp(angular_volts, -12.0, 12.0);

    chassis_.drive_voltage(left_volts, right_volts);

    bool settled = exit_cond_.update(error_rad, dt);
    elapsed_time_ += dt;
    return settled || (elapsed_time_ >= timeout_.to_seconds());
}

void TurnAction::end(bool) {
    chassis_.stop();
}

MoveToPoseAction::MoveToPoseAction(Chassis& chassis, Pose target, Time timeout, double lead)
    : chassis_(chassis),
      target_(target),
      timeout_(timeout),
      lead_(lead),
      lateral_pid_(chassis.get_lateral_gains()),
      angular_pid_(chassis.get_angular_gains()),
      ff_(chassis.get_feedforward_constants()),
      lateral_exit_(0.5, 0.1),
      angular_exit_(1.5 * M_PI / 180.0, 0.1) {}

void MoveToPoseAction::initialize() {
    elapsed_time_ = 0.0;
    lateral_pid_.reset();
    angular_pid_.reset();
    lateral_exit_.reset();
    angular_exit_.reset();
}

bool MoveToPoseAction::update(double dt) {
    Pose current_pose = chassis_.get_odometry()->get_pose();
    double d = current_pose.distance_to(target_).to_inches();

    double target_rad = target_.theta.to_rad();
    double carrot_x = target_.x.to_inches() - lead_ * d * std::cos(target_rad);
    double carrot_y = target_.y.to_inches() - lead_ * d * std::sin(target_rad);
    Pose carrot(Length::inches(carrot_x), Length::inches(carrot_y), target_.theta);

    double distance_to_carrot = current_pose.distance_to(carrot).to_inches();
    double angle_to_carrot = current_pose.angle_to(carrot).to_rad();
    double heading_err = angle_to_carrot - current_pose.theta.to_rad();

    Angle error_angle = Angle::rad(heading_err).constrain();
    double lateral_error = distance_to_carrot * std::cos(error_angle.to_rad());

    double angular_error = 0.0;
    if (d > 0.75) {
        angular_error = error_angle.to_rad();
    } else {
        angular_error = (target_.theta - current_pose.theta).constrain().to_rad();
    }

    double lateral_power = lateral_pid_.update(lateral_error, dt);
    double angular_power = angular_pid_.update(angular_error, dt);

    double lateral_ff = 0.0;
    if (std::abs(lateral_error) > 0.25) {
        lateral_ff = (lateral_error > 0) ? chassis_.get_feedforward_constants().kS : -chassis_.get_feedforward_constants().kS;
    }
    double lateral_volts = lateral_power + lateral_ff;

    double left_volts = std::clamp(lateral_volts - angular_power, -12.0, 12.0);
    double right_volts = std::clamp(lateral_volts + angular_power, -12.0, 12.0);

    chassis_.drive_voltage(left_volts, right_volts);

    double final_heading_err = (target_.theta - current_pose.theta).constrain().to_rad();
    bool settled = lateral_exit_.update(d, dt) && angular_exit_.update(final_heading_err, dt);
    elapsed_time_ += dt;
    return settled || (elapsed_time_ >= timeout_.to_seconds());
}

void MoveToPoseAction::end(bool) {
    chassis_.stop();
}

FollowPathAction::FollowPathAction(Chassis& chassis, std::vector<Pose> path, Length lookahead, Time timeout)
    : chassis_(chassis),
      path_(std::move(path)),
      lookahead_(lookahead),
      timeout_(timeout),
      lateral_pid_(chassis.get_lateral_gains()),
      angular_pid_(chassis.get_angular_gains()),
      ff_(chassis.get_feedforward_constants()),
      exit_cond_(1.0, 0.1) {}

void FollowPathAction::initialize() {
    elapsed_time_ = 0.0;
    lateral_pid_.reset();
    angular_pid_.reset();
    exit_cond_.reset();
}

bool FollowPathAction::update(double dt) {
    if (path_.empty()) return true;

    Pose current_pose = chassis_.get_odometry()->get_pose();

    Pose lookahead_pt = path_.back();
    double L = lookahead_.to_inches();

    for (size_t i = 0; i < path_.size(); ++i) {
        double dist = current_pose.distance_to(path_[i]).to_inches();
        if (dist >= L) {
            lookahead_pt = path_[i];
            break;
        }
    }

    double distance_to_last = current_pose.distance_to(path_.back()).to_inches();

    double angle_to_lookahead = current_pose.angle_to(lookahead_pt).to_rad();
    double heading_err = angle_to_lookahead - current_pose.theta.to_rad();
    Angle error_angle = Angle::rad(heading_err).constrain();

    double alpha = error_angle.to_rad();
    double curvature = (2.0 * std::sin(alpha)) / L;

    double track_w = chassis_.get_config().track_width.to_inches();
    double lateral_power = lateral_pid_.update(distance_to_last, dt);

    double left_power = lateral_power * (1.0 - (curvature * track_w / 2.0));
    double right_power = lateral_power * (1.0 + (curvature * track_w / 2.0));

    double left_volts = std::clamp(left_power, -12.0, 12.0);
    double right_volts = std::clamp(right_power, -12.0, 12.0);

    chassis_.drive_voltage(left_volts, right_volts);

    bool settled = exit_cond_.update(distance_to_last, dt);
    elapsed_time_ += dt;
    return settled || (elapsed_time_ >= timeout_.to_seconds());
}

void FollowPathAction::end(bool) {
    chassis_.stop();
}

MoveRelativeAction::MoveRelativeAction(Chassis& chassis, Length distance, Time timeout)
    : chassis_(chassis), distance_(distance), timeout_(timeout) {}

void MoveRelativeAction::initialize() {
    Pose p0 = chassis_.get_odometry()->get_pose();
    double heading = p0.theta.to_rad();
    double target_x = p0.x.to_inches() + distance_.to_inches() * std::cos(heading);
    double target_y = p0.y.to_inches() + distance_.to_inches() * std::sin(heading);

    move_action_ = std::make_unique<MoveAction>(chassis_, Pose(Length::inches(target_x), Length::inches(target_y), p0.theta), timeout_);
    move_action_->initialize();
}

bool MoveRelativeAction::update(double dt) {
    if (!move_action_) return true;
    return move_action_->update(dt);
}

void MoveRelativeAction::end(bool interrupted) {
    if (move_action_) move_action_->end(interrupted);
}

TurnRelativeAction::TurnRelativeAction(Chassis& chassis, Angle delta_angle, Time timeout)
    : chassis_(chassis), delta_angle_(delta_angle), timeout_(timeout) {}

void TurnRelativeAction::initialize() {
    Pose p0 = chassis_.get_odometry()->get_pose();
    Angle target_heading = (p0.theta + delta_angle_).constrain();

    turn_action_ = std::make_unique<TurnAction>(chassis_, target_heading, timeout_);
    turn_action_->initialize();
}

bool TurnRelativeAction::update(double dt) {
    if (!turn_action_) return true;
    return turn_action_->update(dt);
}

void TurnRelativeAction::end(bool interrupted) {
    if (turn_action_) turn_action_->end(interrupted);
}

} // namespace Vektor
