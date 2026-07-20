#include "Vektor/MCL.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_mcl() {
    using namespace Vektor;

    Pose start_pose(0_in, 0_in, 0_deg);
    ParticleFilter mcl(start_pose);

    Pose est = mcl.get_estimated_pose();
    assert(std::abs(est.x.to_inches() - 0.0) < 3.0);
    assert(std::abs(est.y.to_inches() - 0.0) < 3.0);

    mcl.predict(10.0, 5.0, 0.1);

    Pose est2 = mcl.get_estimated_pose();
    assert(std::abs(est2.x.to_inches() - 10.0) < 4.0);
    assert(std::abs(est2.y.to_inches() - 5.0) < 4.0);

    mcl.set_pose(Pose(12_in, 24_in, 90_deg), 0.1, 0.01);
    Pose est3 = mcl.get_estimated_pose();
    assert(std::abs(est3.x.to_inches() - 12.0) < 1.0);
    assert(std::abs(est3.y.to_inches() - 24.0) < 1.0);
    assert(std::abs(est3.theta.to_deg() - 90.0) < 5.0);

    std::cout << "[PASS] MCL tests passed successfully.\n";
}
