#include "Vektor/Units.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

void test_units() {
    using namespace Vektor;

    Length l1 = 10_in;
    assert(std::abs(l1.to_inches() - 10.0) < 1e-6);
    assert(std::abs(l1.to_meters() - 0.254) < 1e-4);
    assert(std::abs(l1.to_cm() - 25.4) < 1e-4);

    Length l2 = 1_m;
    assert(std::abs(l2.to_inches() - 39.37007874) < 1e-4);

    Length l3 = 100_cm;
    assert(std::abs(l3.to_meters() - 1.0) < 1e-4);

    Length sum = l1 + 5_in;
    assert(std::abs(sum.to_inches() - 15.0) < 1e-6);

    Length diff = sum - 2_in;
    assert(std::abs(diff.to_inches() - 13.0) < 1e-6);

    Length neg = -diff;
    assert(std::abs(neg.to_inches() - (-13.0)) < 1e-6);

    Length scaled = l1 * 2.5;
    assert(std::abs(scaled.to_inches() - 25.0) < 1e-6);

    Length divided = l1 / 2.0;
    assert(std::abs(divided.to_inches() - 5.0) < 1e-6);

    double ratio = scaled / l1;
    assert(std::abs(ratio - 2.5) < 1e-6);

    Length acc(5.0);
    acc += 2_in;
    assert(std::abs(acc.to_inches() - 7.0) < 1e-6);
    acc -= 3_in;
    assert(std::abs(acc.to_inches() - 4.0) < 1e-6);
    acc *= 2.0;
    assert(std::abs(acc.to_inches() - 8.0) < 1e-6);
    acc /= 4.0;
    assert(std::abs(acc.to_inches() - 2.0) < 1e-6);

    assert(10_in == 10_in);
    assert(10_in != 5_in);
    assert(5_in < 10_in);
    assert(5_in <= 5_in);
    assert(10_in > 5_in);
    assert(10_in >= 10_in);

    Angle a1 = 180_deg;
    assert(std::abs(a1.to_rad() - M_PI) < 1e-6);
    assert(std::abs(a1.to_deg() - 180.0) < 1e-6);

    Angle a2 = Angle::rad(M_PI / 2.0);
    assert(std::abs(a2.to_deg() - 90.0) < 1e-6);

    assert(std::abs((360_deg).constrain().to_deg() - 0.0) < 1e-4);
    assert(std::abs((270_deg).constrain().to_deg() - (-90.0)) < 1e-4);
    assert(std::abs((-270_deg).constrain().to_deg() - 90.0) < 1e-4);
    assert(std::abs((540_deg).constrain().to_deg() - 180.0) < 1e-4);

    Time t1 = 2000_ms;
    assert(std::abs(t1.to_seconds() - 2.0) < 1e-6);
    assert(std::abs(t1.to_millis() - 2000.0) < 1e-6);

    Time t2 = 1.5_sec;
    assert(std::abs(t2.to_millis() - 1500.0) < 1e-6);

    Time t_sum = t1 + t2;
    assert(std::abs(t_sum.to_seconds() - 3.5) < 1e-6);

    std::cout << "[PASS] Units tests passed successfully.\n";
}
