#pragma once
#include "Vektor/Action.hpp"
#include "pros/rtos.hpp"
#include <queue>
#include <memory>

namespace Vektor {

class Executor {
public:
    static Executor& get_instance();

    Executor(const Executor&) = delete;
    Executor& operator=(const Executor&) = delete;

    void queue_action(std::unique_ptr<Action> action);
    void cancel_all();
    bool is_running();
    void wait_until_finished();

private:
    Executor();
    ~Executor();

    void run_loop();

    std::unique_ptr<pros::Task> task_;
    std::queue<std::unique_ptr<Action>> queue_;
    std::unique_ptr<Action> active_action_;
    pros::Mutex mutex_;
    bool running_{false};
};

} // namespace Vektor
