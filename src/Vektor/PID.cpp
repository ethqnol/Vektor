#include "Vektor/PID.hpp"
#include <cmath>
#include <algorithm>

namespace Vektor {

PID::PID(Gains gains) : g_(gains) {
    reset();
}

void PID::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
    first_run_ = true;
}

double PID::update(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    double p_out = error * g_.kP;

    if (std::abs(error) <= g_.error_i_thresh) {
        integral_ += error * dt;
        double max_integral_contrib = g_.max_i;
        if (g_.kI != 0.0) {
            double i_limit = max_integral_contrib / g_.kI;
            integral_ = std::clamp(integral_, -i_limit, i_limit);
        }
    } else {
        integral_ = 0.0;
    }
    double i_out = integral_ * g_.kI;

    double d_out = 0.0;
    if (first_run_) {
        prev_error_ = error;
        filtered_derivative_ = 0.0;
        first_run_ = false;
    } else {
        double raw_derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        filtered_derivative_ = (g_.d_filter_alpha * filtered_derivative_) +
                               ((1.0 - g_.d_filter_alpha) * raw_derivative);
        d_out = filtered_derivative_ * g_.kD;
    }

    return std::clamp(p_out + i_out + d_out, -12.0, 12.0);
}

} // namespace Vektor
