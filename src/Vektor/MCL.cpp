#include "Vektor/MCL.hpp"
#include <cmath>
#include <algorithm>

namespace Vektor {

ParticleFilter::ParticleFilter(Pose initial_pose,
                               std::shared_ptr<pros::Distance> distance_sensor,
                               Pose sensor_offset)
    : distance_sensor_(distance_sensor),
      sensor_offset_(sensor_offset),
      rng_(std::random_device{}()) {
    set_pose(initial_pose);
}

void ParticleFilter::set_pose(Pose pose, double std_xy, double std_theta) {
    std::normal_distribution<double> init_x(pose.x.to_inches(), std_xy);
    std::normal_distribution<double> init_y(pose.y.to_inches(), std_xy);
    std::normal_distribution<double> init_theta(pose.theta.to_rad(), std_theta);

    for (auto& p : particles_) {
        p.x = init_x(rng_);
        p.y = init_y(rng_);
        p.theta = Angle::rad(init_theta(rng_)).constrain().to_rad();
        p.weight = 1.0 / static_cast<double>(N_PARTICLES);
    }
}

void ParticleFilter::predict(double dx, double dy, double dtheta) {
    for (auto& p : particles_) {
        double noise_x = noise_x_(rng_);
        double noise_y = noise_y_(rng_);
        double noise_th = noise_theta_(rng_);

        p.x += dx + noise_x;
        p.y += dy + noise_y;
        p.theta = Angle::rad(p.theta + dtheta + noise_th).constrain().to_rad();
    }
}

double ParticleFilter::raycast_wall_distance(double px, double py, double ptheta) const {
    double ux = std::cos(ptheta);
    double uy = std::sin(ptheta);

    double min_dist = 1e6;

    // east wall (x = +72)
    if (ux > 1e-4) {
        double d = (72.0 - px) / ux;
        if (d > 0 && d < min_dist) min_dist = d;
    }
    // west wall (x = -72)
    if (ux < -1e-4) {
        double d = (-72.0 - px) / ux;
        if (d > 0 && d < min_dist) min_dist = d;
    }
    // north wall (y = +72)
    if (uy > 1e-4) {
        double d = (72.0 - py) / uy;
        if (d > 0 && d < min_dist) min_dist = d;
    }
    // south wall (y = -72)
    if (uy < -1e-4) {
        double d = (-72.0 - py) / uy;
        if (d > 0 && d < min_dist) min_dist = d;
    }

    return min_dist;
}

void ParticleFilter::update_sensor() {
    if (!distance_sensor_) return;

    int raw_mm = distance_sensor_->get();
    if (raw_mm <= 0 || raw_mm > 4000) return; // ignore invalid or out of range readings

    double z_meas = Length::cm(raw_mm / 10.0).to_inches();
    double sigma_sensor = 1.5;

    double sum_weights = 0.0;
    for (auto& p : particles_) {
        double sensor_angle = p.theta + sensor_offset_.theta.to_rad();
        double sx = p.x + sensor_offset_.x.to_inches() * std::cos(p.theta) - sensor_offset_.y.to_inches() * std::sin(p.theta);
        double sy = p.y + sensor_offset_.x.to_inches() * std::sin(p.theta) + sensor_offset_.y.to_inches() * std::cos(p.theta);

        double z_exp = raycast_wall_distance(sx, sy, sensor_angle);
        double error = z_meas - z_exp;

        p.weight = std::exp(-(error * error) / (2.0 * sigma_sensor * sigma_sensor));
        sum_weights += p.weight;
    }

    if (sum_weights > 1e-9) {
        for (auto& p : particles_) {
            p.weight /= sum_weights;
        }
        resample();
    }
}

void ParticleFilter::resample() {
    std::array<Particle, N_PARTICLES> new_particles;
    std::uniform_real_distribution<double> dist(0.0, 1.0 / static_cast<double>(N_PARTICLES));

    double r = dist(rng_);
    double c = particles_[0].weight;
    size_t i = 0;

    for (size_t m = 0; m < N_PARTICLES; ++m) {
        double U = r + static_cast<double>(m) / static_cast<double>(N_PARTICLES);
        while (U > c && i < N_PARTICLES - 1) {
            i++;
            c += particles_[i].weight;
        }
        new_particles[m] = particles_[i];
        new_particles[m].weight = 1.0 / static_cast<double>(N_PARTICLES);
    }

    particles_ = new_particles;
}

Pose ParticleFilter::get_estimated_pose() {
    double mean_x = 0.0;
    double mean_y = 0.0;
    double sin_sum = 0.0;
    double cos_sum = 0.0;

    for (const auto& p : particles_) {
        mean_x += p.x * p.weight;
        mean_y += p.y * p.weight;
        sin_sum += std::sin(p.theta) * p.weight;
        cos_sum += std::cos(p.theta) * p.weight;
    }

    double mean_theta = std::atan2(sin_sum, cos_sum);
    return Pose(Length::inches(mean_x), Length::inches(mean_y), Angle::rad(mean_theta));
}

} // namespace Vektor
