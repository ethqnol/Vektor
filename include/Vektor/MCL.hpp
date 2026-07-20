#pragma once
#include "Vektor/Pose.hpp"
#include "Vektor/Units.hpp"
#include "pros/distance.hpp"
#include <array>
#include <memory>
#include <random>

namespace Vektor {

struct Particle {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double weight{0.01};
};

class ParticleFilter {
public:
    static constexpr size_t N_PARTICLES = 100;

    ParticleFilter(Pose initial_pose,
                   std::shared_ptr<pros::Distance> distance_sensor = nullptr,
                   Pose sensor_offset = Pose(0_in, 0_in, 0_deg));

    void predict(double dx, double dy, double dtheta);
    void update_sensor();
    void resample();
    Pose get_estimated_pose();
    void set_pose(Pose pose, double std_xy = 2.0, double std_theta = 0.05);

private:
    double raycast_wall_distance(double px, double py, double ptheta) const;

    std::array<Particle, N_PARTICLES> particles_;
    std::shared_ptr<pros::Distance> distance_sensor_;
    Pose sensor_offset_;

    std::mt19937 rng_;
    std::normal_distribution<double> noise_x_{0.0, 0.15};
    std::normal_distribution<double> noise_y_{0.0, 0.15};
    std::normal_distribution<double> noise_theta_{0.0, 0.01};
};

} // namespace Vektor
