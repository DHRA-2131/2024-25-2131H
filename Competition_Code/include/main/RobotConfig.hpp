#pragma once

#include "lemlib/api.hpp"
#include "path-addin/ChassisAddin.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"

// Constants
constexpr double driveRPM = 450;
constexpr double trackWidth = 13.5;
constexpr double armHeight = 300;

// Team Color Toggle
namespace Screen
{
// Team Color Button
extern pros::adi::DigitalIn teamColor;
}  // namespace Screen

namespace Systems
{
namespace Drivetrain
{
// Left and Right drive
extern pros::v5::MotorGroup leftDrive;
extern pros::v5::MotorGroup rightDrive;

// Vex V5 Inertial for heading
extern pros::Imu imu;
}  // namespace Drivetrain

namespace Clamp
{
// Pneumatics for clamp
extern pros::adi::Pneumatics pneumatic;

// Distance sensor for detecting goals
extern pros::Distance goalDetector;
}  // namespace Clamp

namespace Arm
{
// Arm Motor
extern pros::v5::Motor motor;
extern pros::v5::Rotation rotational;

// Doinkler (for removing corner rings)
extern pros::adi::Pneumatics doinkler;
};  // namespace Arm

namespace Intake
{
extern pros::v5::Motor motor;           // Intake Motors
extern pros::Optical colorDetector;     // Color Detector
extern pros::Distance ringDetector;     // Vex V5 Distance Sensor (For detecting rings as they approach the top of the intake)
extern pros::adi::Pneumatics ringSort;  // Pneumatic for ejecting rings from intake

}  // namespace Intake
}  // namespace Systems

// Controllers
extern pros::Controller primary;  // Primary Controller

// *** === LEMLIB Utilization === *** //
// Drivetrain
extern lemlib::Drivetrain drivetrain;  //

// Odometry Config
extern lemlib::TrackingWheel leftTrackingWheel;   // Tracking wheels for odom
extern lemlib::TrackingWheel rightTrackingWheel;  // Tracking wheels for odom
extern lemlib::OdomSensors sensors;               // Tracking wheels + IMU

// PID Controllers
extern lemlib::ControllerSettings lateral_controller;  // Movement PIDs
extern lemlib::ControllerSettings angular_controller;  // Movement PIDs

// Chassis Class
extern Pathing::Chassis chassis;  // Chassis class
