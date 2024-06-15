/**
 * @file robot-config.cpp
 * @author Andrew Hilton (2131H)
 * @brief Robot Config Src
 * @version 0.1
 * @date 2024-06-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "main/robot-config.hpp"

#include "pros/adi.hpp"

// ODOM BOT DRIVE PORTS
// -4, 1, -2 LF, LM, LR
// 5, 8, 6 RF, RM, RR

// Drive Motors
pros::Motor LeftFront(-4, pros::v5::MotorGearset::green);
pros::Motor LeftMid(-1, pros::v5::MotorGearset::green);
pros::Motor LeftRear(-2, pros::v5::MotorGearset::green);

pros::Motor RightFront(5, pros::v5::MotorGearset::green);
pros::Motor RightMid(8, pros::v5::MotorGearset::green);
pros::Motor RightRear(6, pros::v5::MotorGearset::green);

pros::v5::MotorGroup LeftDrive({-4, -1, -2}, pros::v5::MotorGearset::green);
pros::v5::MotorGroup RightDrive({5, 8, 6}, pros::v5::MotorGearset::green);

// Pneumatics
pros::adi::Pneumatics Clamp(1, true, false);

// Sensors
pros::Rotation RearEncoder(10);
pros::Rotation LeftEncoder(-19);
pros::Rotation RightEncoder(20);

pros::IMU Inertial(21);

// Tracking Wheels
lib2131::TrackingWheel RearWheel(&RearEncoder, 2.0, 5);
lib2131::TrackingWheel LeftDeadWheel(&LeftEncoder, 2.0, 7.0 / 2.0);
lib2131::TrackingWheel RightDeadWheel(&RightEncoder, 2.0, -7.0 / 2.0);

// Drivetrain T-Wheels
lib2131::TrackingWheel LeftDriveWheel(&LeftDrive, 3.25, 12.75 / 2.0, 333 + (1.0 / 3.0));
lib2131::TrackingWheel RightDriveWheel(&RightDrive, 3.25, -12.75 / 2.0, 333 + (1.0 / 3));

// Odometry
lib2131::Odometry TWheelOdom(&LeftDeadWheel, &RightDeadWheel, &RearWheel, nullptr);
lib2131::Odometry DriveOdom(&LeftDriveWheel, &RightDriveWheel, &RearWheel, &Inertial);

lib2131::PID linearPID(1, 0, 0);
lib2131::PID angularPID(1, 0, 0);

// Robot
lib2131::Robot Robot(9999.0, -9999.0, 56.68, {&TWheelOdom, &DriveOdom}, angularPID,
                     linearPID, &LeftDrive, &RightDrive);

// Controller
pros::Controller primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);