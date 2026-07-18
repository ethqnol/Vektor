#pragma once

namespace Vektor {

class PID {
public:
    struct Gains {
        double kP{0.0};
        double kI{0.0};
        double kD{0.0};
        double max_i{12.0};         // anti-windup cap in volts
        double error_i_thresh{2.0}; // active integral error band
        double d_filter_alpha{0.8}; // derivative low-pass filter factor (0.0 to 1.0)
    };

    explicit PID(Gains gains);

    double update(double error, double dt);
    void reset();

private:
    Gains g_;
    double integral_{0.0};
    double prev_error_{0.0};
    double filtered_derivative_{0.0};
    bool first_run_{true};
};

} // namespace Vektor
