# Vektor Robotics Library

**Vektor** is a high-performance, hand-crafted C++20 drivetrain, odometry, and autonomous motion library designed for PROS on VEX V5 hardware.

---

## Key Features

* **Strongly-Typed Physical Units**: Type-safe length, angle, and time representations (`_in`, `_m`, `_cm`, `_deg`, `_rad`, `_sec`, `_ms`).
* **500Hz Circular Arc Odometry**: Non-linear arc tracking with automatic fallback to motor encoders if the IMU disconnects or reports invalid data.
* **Monte Carlo Localization (MCL)**: Zero-allocation particle filter (`std::array<Particle, 100>`) with wall distance raycasting for sensor fusion.
* **Autonomous Motion Controllers**:
  * Point turns with constrained angle wrapping.
  * Boomerang curve pose alignment (`MoveToPoseAction`).
  * Pure Pursuit path following (`FollowPathAction`).
  * Non-blocking spatial triggers & event callbacks (`add_spatial_trigger`).
* **Absolute vs. Relative Field Mapping**: Seamlessly switch between field-centric absolute coordinates ($(0, 0)$ at center of field) and robot-relative starting coordinates.
* **Driver Control (Opcontrol)**:
  * Arcade, Tank, and Cheesy Curvature steering modes.
  * Exponential joystick response curves (`DriveCurve`) with customizable deadbands and voltage desaturation.

---

## Deployment & Installation

### Option 1: PROS Template Installation (Recommended)

Generate the Vektor template package and apply it to any PROS project:

```bash
# 1. Package Vektor into a PROS template
cd /home/ewu/Code/Vektor
pros c create-template . Vektor 1.0.0 --target v5 --system "include/Vektor/*.hpp" --system "src/Vektor/*.cpp"

# 2. Fetch the template into PROS depot
pros c fetch Vektor@1.0.0.zip

# 3. Apply to your robot project
cd /path/to/your_pros_project
pros c apply Vektor
```

### Option 2: Direct Source Copy

Copy the source files directly into your robot project:

```bash
cp -r /home/ewu/Code/Vektor/include/Vektor /path/to/your_pros_project/include/
cp -r /home/ewu/Code/Vektor/src/Vektor /path/to/your_pros_project/src/
```

---

## Usage Example (`src/main.cpp`)

```cpp
#include "main.h"
#include "Vektor/Chassis.hpp"
#include "Vektor/Odometry.hpp"

using namespace Vektor;

static std::shared_ptr<pros::MotorGroup> left_motors;
static std::shared_ptr<pros::MotorGroup> right_motors;
static std::shared_ptr<TrackingWheel> forward_wheel;
static std::shared_ptr<TrackingWheel> lateral_wheel;
static std::shared_ptr<pros::IMU> imu;
static std::shared_ptr<Odometry> odom;
static std::unique_ptr<Chassis> chassis;

void initialize() {
    left_motors = std::make_shared<pros::MotorGroup>(std::vector<int8_t>{1, -2, 3});
    right_motors = std::make_shared<pros::MotorGroup>(std::vector<int8_t>{-4, 5, -6});

    auto forward_rot = std::make_shared<pros::Rotation>(7);
    auto lateral_rot = std::make_shared<pros::Rotation>(8);

    forward_wheel = std::make_shared<TrackingWheel>(forward_rot, 2.0_in, 1.0, 0.0_in);
    lateral_wheel = std::make_shared<TrackingWheel>(lateral_rot, 2.0_in, 1.0, -2.5_in);
    imu = std::make_shared<pros::IMU>(9);

    odom = std::make_shared<Odometry>(
        forward_wheel, lateral_wheel, imu,
        left_motors, right_motors, 12.5_in, 3.25_in, 1.0
    );

    DrivetrainConfig config{3.25_in, 12.5_in, 1.0};
    PID::Gains lateral_pid{2.5, 0.0, 0.15, 12.0, 3.0, 0.8};
    PID::Gains angular_pid{3.2, 0.0, 0.22, 12.0, 5.0, 0.8};
    Feedforward::Constants ff{1.1, 0.45, 0.08};

    chassis = std::make_unique<Chassis>(
        left_motors, right_motors, odom,
        config, lateral_pid, angular_pid, ff
    );

    chassis->set_starting_pose(Pose(-48_in, 12_in, 90_deg));
    odom->start();
}

void autonomous() {
    // Chain autonomous actions using SequenceBuilder
    chassis->new_sequence()
        .move_to(Pose(-24_in, 24_in, 45_deg), 2.5_sec, CoordinateFrame::ABSOLUTE)
        .turn_to(180_deg, 1.2_sec)
        .move_to(Pose(12_in, -6_in, 0_deg), 2.0_sec, CoordinateFrame::RELATIVE)
        .execute();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        double throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis->arcade(throttle, turn);
        pros::delay(10);
    }
}
```

---

## Native Unit Testing

Run the C++ unit test suite locally:

```bash
make test
```
