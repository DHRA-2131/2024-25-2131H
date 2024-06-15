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

#include "lib2131/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// Motors + MotorGroups
extern pros::Motor LeftFront;
extern pros::Motor LeftMid;
extern pros::Motor LeftRear;

extern pros::Motor RightFront;
extern pros::Motor RightMid;
extern pros::Motor RightRear;

extern pros::v5::MotorGroup LeftDrive;
extern pros::v5::MotorGroup RightDrive;

// Pneumatics
extern pros::adi::Pneumatics Clamp;

// Sensors
extern pros::Rotation RearEncoder;
extern pros::Rotation LeftEncoder;
extern pros::Rotation RightEncoder;

extern pros::IMU Inertial;

// Tracking Wheels
extern lib2131::TrackingWheel RearWheel;
extern lib2131::TrackingWheel LeftDeadWheel;
extern lib2131::TrackingWheel RightDeadWheel;

// Drivetrain T-Wheels
extern lib2131::TrackingWheel LeftDriveWheel;
extern lib2131::TrackingWheel RightDriveWheel;

// Odometry
extern lib2131::Odometry TWheelOdom;
extern lib2131::Odometry DriveOdom;

extern lib2131::PID linearPID;
extern lib2131::PID angularPID;

// Robot
extern lib2131::Robot Robot;

// Controller
extern pros::Controller primary;