#pragma once
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

// Constants
constexpr double driveRPM = 450;
constexpr double trackWidth = 13.5;
constexpr double armHeight = 300;

// Team Color Toggle
namespace Screen
{
extern pros::adi::DigitalIn teamColor;
}

namespace Systems
{
namespace Drivetrain
{
extern pros::v5::MotorGroup leftDrive;
extern pros::v5::MotorGroup rightDrive;
extern pros::Imu imu;
}  // namespace Drivetrain

namespace Clamp
{
extern pros::adi::Pneumatics pneumatic;
extern pros::Distance goalDetector;
}  // namespace Clamp

namespace Arm
{
extern pros::v5::Motor motor;
extern pros::adi::Pneumatics doinkler;
};  // namespace Arm

namespace Intake
{
extern pros::v5::Motor motor;
extern pros::Optical colorDetector;
extern pros::Distance ringDetector;
}  // namespace Intake
}  // namespace Systems

// Controller
extern pros::Controller primary;

// LEMLIB
// Drivetrain
extern lemlib::Drivetrain drivetrain;

// Odometry Config
extern lemlib::TrackingWheel leftTrackingWheel;
extern lemlib::TrackingWheel rightTrackingWheel;
extern lemlib::OdomSensors sensors;

// PID Controllers
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

// Chassis Class
extern lemlib::Chassis chassis;
