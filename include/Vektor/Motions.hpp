#pragma once
#include "Vektor/Action.hpp"
#include "Vektor/Pose.hpp"
#include "Vektor/PID.hpp"
#include "Vektor/Feedforward.hpp"
#include <cmath>
#include <vector>
#include <memory>

namespace Vektor {

class Chassis;

class ExitCondition {
public:
    ExitCondition(double range, double settle_time)
        : range_(range), settle_time_(settle_time), time_settled_(0.0) {}

    bool update(double error, double dt) {
        if (std::abs(error) <= range_) {
            time_settled_ += dt;
        } else {
            time_settled_ = 0.0;
        }
        return time_settled_ >= settle_time_;
    }

    void reset() { time_settled_ = 0.0; }

private:
    double range_;
    double settle_time_;
    double time_settled_;
};

class MoveAction : public Action {
public:
    MoveAction(Chassis& chassis, Pose target, Time timeout);

    void initialize() override;
    bool update(double dt) override;
    void end(bool interrupted) override;

private:
    Chassis& chassis_;
    Pose target_;
    Time timeout_;
    double elapsed_time_{0.0};

    PID lateral_pid_;
    PID angular_pid_;
    Feedforward ff_;

    ExitCondition lateral_exit_;
    ExitCondition angular_exit_;
};

class TurnAction : public Action {
public:
    TurnAction(Chassis& chassis, Angle target_heading, Time timeout);

    void initialize() override;
    bool update(double dt) override;
    void end(bool interrupted) override;

private:
    Chassis& chassis_;
    Angle target_heading_;
    Time timeout_;
    double elapsed_time_{0.0};

    PID pid_;
    ExitCondition exit_cond_;
};

class MoveToPoseAction : public Action {
public:
    MoveToPoseAction(Chassis& chassis, Pose target, Time timeout, double lead = 0.6);

    void initialize() override;
    bool update(double dt) override;
    void end(bool interrupted) override;

private:
    Chassis& chassis_;
    Pose target_;
    Time timeout_;
    double lead_;
    double elapsed_time_{0.0};

    PID lateral_pid_;
    PID angular_pid_;
    Feedforward ff_;

    ExitCondition lateral_exit_;
    ExitCondition angular_exit_;
};

class FollowPathAction : public Action {
public:
    FollowPathAction(Chassis& chassis, std::vector<Pose> path, Length lookahead, Time timeout);

    void initialize() override;
    bool update(double dt) override;
    void end(bool interrupted) override;

private:
    Chassis& chassis_;
    std::vector<Pose> path_;
    Length lookahead_;
    Time timeout_;
    double elapsed_time_{0.0};

    PID lateral_pid_;
    PID angular_pid_;
    Feedforward ff_;

    ExitCondition exit_cond_;
};

class MoveRelativeAction : public Action {
public:
    MoveRelativeAction(Chassis& chassis, Length distance, Time timeout);

    void initialize() override;
    bool update(double dt) override;
    void end(bool interrupted) override;

private:
    Chassis& chassis_;
    Length distance_;
    Time timeout_;
    std::unique_ptr<MoveAction> move_action_;
};

class TurnRelativeAction : public Action {
public:
    TurnRelativeAction(Chassis& chassis, Angle delta_angle, Time timeout);

    void initialize() override;
    bool update(double dt) override;
    void end(bool interrupted) override;

private:
    Chassis& chassis_;
    Angle delta_angle_;
    Time timeout_;
    std::unique_ptr<TurnAction> turn_action_;
};

} // namespace Vektor
