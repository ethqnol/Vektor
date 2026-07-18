#include "Vektor/Feedforward.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_feedforward() {
    using namespace Vektor;

    Feedforward::Constants consts;
    consts.kS = 1.0;
    consts.kV = 0.5;
    consts.kA = 0.1;

    Feedforward ff(consts);

    double v1 = ff.calculate(10.0, 2.0);
    assert(std::abs(v1 - 6.2) < 1e-6);

    double v2 = ff.calculate(-10.0, -2.0);
    assert(std::abs(v2 - (-6.2)) < 1e-6);

    double v3 = ff.calculate(0.0, 0.0);
    assert(std::abs(v3 - 0.0) < 1e-6);

    std::cout << "[PASS] Feedforward tests passed successfully.\n";
}
