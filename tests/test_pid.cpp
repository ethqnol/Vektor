#include "Vektor/PID.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_pid() {
    using namespace Vektor;

    PID::Gains gains;
    gains.kP = 2.0;
    gains.kI = 0.5;
    gains.kD = 0.1;
    gains.max_i = 5.0;
    gains.error_i_thresh = 10.0;
    gains.d_filter_alpha = 0.5;

    PID pid(gains);

    double out1 = pid.update(5.0, 0.01);
    assert(out1 > 0.0);

    for (int i = 0; i < 5; ++i) {
        double out = pid.update(5.0, 0.01);
        assert(out > 0.0);
    }

    double out_large_error = pid.update(20.0, 0.01);
    assert(std::abs(out_large_error) <= 12.0);

    pid.reset();
    double out_after_reset = pid.update(5.0, 0.01);
    assert(std::abs(out_after_reset - out1) < 1e-4);

    std::cout << "[PASS] PID tests passed successfully.\n";
}
