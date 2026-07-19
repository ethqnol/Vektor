#include "Vektor/Action.hpp"
#include "Vektor/Motions.hpp"
#include <cassert>
#include <iostream>
#include <memory>
#include <vector>

class MockStepAction : public Vektor::Action {
public:
    explicit MockStepAction(int steps_to_finish) : steps_(steps_to_finish) {}

    void initialize() override { count_ = 0; }
    bool update(double) override {
        count_++;
        return count_ >= steps_;
    }

private:
    int steps_;
    int count_{0};
};

void test_actions() {
    using namespace Vektor;

    ExitCondition exit_cond(0.5, 0.1);

    assert(!exit_cond.update(0.2, 0.05));
    assert(exit_cond.update(0.2, 0.06));

    assert(!exit_cond.update(1.0, 0.05));
    assert(!exit_cond.update(0.2, 0.05));

    std::vector<std::unique_ptr<Action>> seq_list;
    seq_list.push_back(std::make_unique<MockStepAction>(2));
    seq_list.push_back(std::make_unique<MockStepAction>(3));

    SequentialAction seq(std::move(seq_list));
    seq.initialize();

    assert(!seq.update(0.01));
    assert(!seq.update(0.01));
    assert(!seq.update(0.01));
    assert(!seq.update(0.01));
    assert(seq.update(0.01));

    std::vector<std::unique_ptr<Action>> race_list;
    race_list.push_back(std::make_unique<MockStepAction>(2));
    race_list.push_back(std::make_unique<MockStepAction>(10));

    RaceAction race(std::move(race_list));
    race.initialize();

    assert(!race.update(0.01));
    assert(race.update(0.01));

    std::cout << "[PASS] Action & ExitCondition tests passed successfully.\n";
}
