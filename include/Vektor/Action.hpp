#pragma once
#include <vector>
#include <memory>

namespace Vektor {

class Action {
public:
    virtual ~Action() = default;

    virtual void initialize() {}
    virtual bool update(double dt) = 0;
    virtual void end(bool) {}
    
    virtual double get_exit_velocity() const { return 0.0; }
};

class SequentialAction : public Action {
public:
    explicit SequentialAction(std::vector<std::unique_ptr<Action>> actions)
        : actions_(std::move(actions)) {}

    void initialize() override {
        index_ = 0;
        if (!actions_.empty()) {
            actions_[index_]->initialize();
        }
    }

    bool update(double dt) override {
        if (actions_.empty()) return true;

        if (actions_[index_]->update(dt)) {
            actions_[index_]->end(false);
            index_++;
            if (index_ >= actions_.size()) {
                return true;
            }
            actions_[index_]->initialize();
        }
        return false;
    }

    void end(bool interrupted) override {
        if (interrupted && index_ < actions_.size()) {
            actions_[index_]->end(true);
        }
    }

private:
    std::vector<std::unique_ptr<Action>> actions_;
    size_t index_{0};
};

class ParallelAction : public Action {
public:
    explicit ParallelAction(std::vector<std::unique_ptr<Action>> actions)
        : actions_(std::move(actions)) {}

    void initialize() override {
        finished_.assign(actions_.size(), false);
        for (auto& action : actions_) {
            action->initialize();
        }
    }

    bool update(double dt) override {
        bool all_finished = true;
        for (size_t i = 0; i < actions_.size(); ++i) {
            if (!finished_[i]) {
                if (actions_[i]->update(dt)) {
                    actions_[i]->end(false);
                    finished_[i] = true;
                } else {
                    all_finished = false;
                }
            }
        }
        return all_finished;
    }

    void end(bool interrupted) override {
        if (interrupted) {
            for (size_t i = 0; i < actions_.size(); ++i) {
                if (!finished_[i]) {
                    actions_[i]->end(true);
                }
            }
        }
    }

private:
    std::vector<std::unique_ptr<Action>> actions_;
    std::vector<bool> finished_;
};

class RaceAction : public Action {
public:
    explicit RaceAction(std::vector<std::unique_ptr<Action>> actions)
        : actions_(std::move(actions)) {}

    void initialize() override {
        finished_ = false;
        for (auto& action : actions_) {
            action->initialize();
        }
    }

    bool update(double dt) override {
        for (auto& action : actions_) {
            if (action->update(dt)) {
                finished_ = true;
                return true;
            }
        }
        return false;
    }

    void end(bool interrupted) override {
        for (auto& action : actions_) {
            action->end(interrupted || finished_);
        }
    }

private:
    std::vector<std::unique_ptr<Action>> actions_;
    bool finished_{false};
};

} // namespace Vektor
