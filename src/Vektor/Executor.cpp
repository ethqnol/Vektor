#include "Vektor/Executor.hpp"

namespace Vektor {

Executor& Executor::get_instance() {
    static Executor instance;
    return instance;
}

Executor::Executor() : running_(true) {
    task_ = std::make_unique<pros::Task>([this]() {
        run_loop();
    }, "VektorExecutor");
}

Executor::~Executor() {
    {
        std::lock_guard<pros::Mutex> lock(mutex_);
        running_ = false;
    }
    task_.reset();
}

void Executor::queue_action(std::unique_ptr<Action> action) {
    if (!action) return;
    std::lock_guard<pros::Mutex> lock(mutex_);
    queue_.push(std::move(action));
}

void Executor::cancel_all() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    if (active_action_) {
        active_action_->end(true);
        active_action_.reset();
    }
    while (!queue_.empty()) {
        queue_.pop();
    }
}

bool Executor::is_running() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return active_action_ != nullptr || !queue_.empty();
}

void Executor::wait_until_finished() {
    while (is_running()) {
        pros::delay(10);
    }
}

void Executor::run_loop() {
    std::uint32_t prev_time = pros::millis();

    while (true) {
        {
            std::lock_guard<pros::Mutex> lock(mutex_);
            if (!running_) break;

            std::uint32_t now = pros::millis();
            double dt = static_cast<double>(now - prev_time) / 1000.0;
            if (dt > 0.1) dt = 0.1;
            prev_time = now;

            if (!active_action_ && !queue_.empty()) {
                active_action_ = std::move(queue_.front());
                queue_.pop();
                active_action_->initialize();
            }

            if (active_action_) {
                bool done = active_action_->update(dt);
                if (done) {
                    active_action_->end(false);
                    active_action_.reset();
                }
            }
        }

        pros::delay(10);
    }
}

} // namespace Vektor
