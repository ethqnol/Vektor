#pragma once

namespace Vektor {

class Feedforward {
public:
    struct Constants {
        double kS{0.0}; // volts to break static friction
        double kV{0.0}; // volts / (inch/sec)
        double kA{0.0}; // volts / (inch/sec^2)
    };

    explicit Feedforward(Constants constants) : c_(constants) {}

    double calculate(double velocity, double acceleration) const {
        double static_offset = (velocity > 0.0) ? c_.kS : ((velocity < 0.0) ? -c_.kS : 0.0);
        return static_offset + (c_.kV * velocity) + (c_.kA * acceleration);
    }

private:
    Constants c_;
};

} // namespace Vektor
