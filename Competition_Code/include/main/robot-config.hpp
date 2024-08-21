/**
 * @file robot-config.cpp
 * @author Andrew Hilton (2131H)
 * @brief Robot Configuration
 * @version 0.1
 * @date 2024-06-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

#define LEFT_DRIVE_PORTS {2}
#define RIGHT_DRIVE_PORTS {-1}
#define WHEEL_SIZE 2.75
#define WHEEL_RPM 200
// Motor Groups

extern pros::v5::MotorGroup LeftDrive;
extern pros::v5::MotorGroup RightDrive;

// Pneumatics
extern pros::adi::Pneumatics Clamp;

// Sensors
extern pros::IMU Inertial;

// using namespace lib2131::odometry::trackingWheel

// // Dead Wheels
// extern std::shared_ptr<RotationalTrackingWheel> LeftDeadWheel;
// extern std::shared_ptr<RotationalTrackingWheel> RightDeadWheel;
// extern std::shared_ptr<RotationalTrackingWheel> RearWheel;

// // Motor Encoder Tracking Wheels
// extern std::shared_ptr<MotorTrackingWheel> LeftDrivenWheel;
// extern std::shared_ptr<MotorTrackingWheel> RightDrivenWheel;

// // Odometry
// extern std::shared_ptr<lib2131::odometry::BlendedOdometry> DrivenOdom;
// extern std::shared_ptr<lib2131::odometry::WheelOdometry> DeadOdom;

// Controller
extern pros::Controller primary;