#include "Vektor/Pose.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_pose() {
    using namespace Vektor;

    Pose p0(0_in, 0_in, 0_deg);
    Pose p_q1(3_in, 4_in, 0_deg);

    assert(std::abs(p0.distance_to(p_q1).to_inches() - 5.0) < 1e-6);
    assert(std::abs(p0.angle_to(p_q1).to_deg() - 53.13010235) < 1e-3);

    Pose p_q2(-4_in, 4_in, 0_deg);
    assert(std::abs(p0.angle_to(p_q2).to_deg() - 135.0) < 1e-3);

    Pose p_q3(-3_in, -4_in, 0_deg);
    assert(std::abs(p0.distance_to(p_q3).to_inches() - 5.0) < 1e-6);

    Pose p_q4(3_in, -3_in, 0_deg);
    assert(std::abs(p0.angle_to(p_q4).to_deg() - (-45.0)) < 1e-3);

    assert(std::abs(p0.distance_to(p0).to_inches() - 0.0) < 1e-6);

    // Coordinate Frame Transformations (Absolute vs Relative)
    // Starting pose on field: (-48", 12", 90 deg)
    Pose absolute_start(-48_in, 12_in, 90_deg);

    // Relative movement of (10", 0", 0 deg) forward from start
    Pose relative_move(10_in, 0_in, 0_deg);
    Pose abs_result = relative_move.to_absolute(absolute_start);

    // Moving 10" forward when facing 90 deg (North) should increase Y by 10" -> (-48", 22", 90 deg)
    assert(std::abs(abs_result.x.to_inches() - (-48.0)) < 1e-4);
    assert(std::abs(abs_result.y.to_inches() - 22.0) < 1e-4);
    assert(std::abs(abs_result.theta.to_deg() - 90.0) < 1e-4);

    // Converting absolute pose (-48", 22", 90 deg) back to relative coordinates should give (10", 0", 0 deg)
    Pose rel_back = abs_result.to_relative(absolute_start);
    assert(std::abs(rel_back.x.to_inches() - 10.0) < 1e-4);
    assert(std::abs(rel_back.y.to_inches() - 0.0) < 1e-4);
    assert(std::abs(rel_back.theta.to_deg() - 0.0) < 1e-4);

    std::cout << "[PASS] Pose & CoordinateFrame tests passed successfully.\n";
}
