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
// TODO: add carts to motor odom
#include "main/robot-config.hpp"

#include <memory>

#include "lib2131/odometry/OdometryBuilder.hpp"
#include "lib2131/odometry/WheelOdometry.hpp"
#include "lib2131/odometry/tracking_wheel/MotorTrackingWheel.hpp"
#include "lib2131/odometry/tracking_wheel/RotationalTrackingWheel.hpp"
#include "lib2131/odometry/tracking_wheel/TrackingWheelBuilder.hpp"
#include "pros/adi.hpp"

// ODOM BOT DRIVE PORTS
// -4, 1, -2 LF, LM, LR
// 5, 8, 6 RF, RM, RR

using namespace units::literals;

#define LEFT_DRIVE_PORTS {-4, -1, -2}
#define RIGHT_DRIVE_PORTS {5, 8, 6}

// Drive Motors

pros::v5::MotorGroup LeftDrive(LEFT_DRIVE_PORTS, pros::v5::MotorGearset::green);
pros::v5::MotorGroup RightDrive(RIGHT_DRIVE_PORTS, pros::v5::MotorGearset::green);

// Pneumatics
pros::adi::Pneumatics Clamp(1, true, false);

// Sensors
pros::IMU Inertial(21);

// Dead Wheels
std::shared_ptr<RotationalTrackingWheel> LeftDeadWheel =
    lib2131::odometry::TrackingWheelBuilder::newBuilder()
        .setWheelDiameter(2.0_in)
        .setOffset(3.5_in)
        .buildRotationalTrackingWheel(20);

std::shared_ptr<RotationalTrackingWheel> RightDeadWheel =
    lib2131::odometry::TrackingWheelBuilder::newBuilder()
        .setWheelDiameter(2.0_in)
        .setOffset(-3.5_in)
        .buildRotationalTrackingWheel(-19);

std::shared_ptr<RotationalTrackingWheel> RearWheel =
    lib2131::odometry::TrackingWheelBuilder::newBuilder()
        .setWheelDiameter(2.0_in)
        .setOffset(5_in)
        .buildRotationalTrackingWheel(10);

// Motor Encoder Tracking Wheels
std::shared_ptr<MotorTrackingWheel> LeftDrivenWheel =
    lib2131::odometry::TrackingWheelBuilder::newBuilder()
        .setWheelDiameter(3.25_in)
        .setOffset(6.375_in)
        .buildMotorTrackingWheel(LEFT_DRIVE_PORTS, 333.0 + (1.0 / 3.0));

std::shared_ptr<MotorTrackingWheel> RightDrivenWheel =
    lib2131::odometry::TrackingWheelBuilder::newBuilder()
        .setWheelDiameter(3.25_in)
        .setOffset(-6.375_in)
        .buildMotorTrackingWheel(RIGHT_DRIVE_PORTS, 333.0 + (1.0 / 3.0));

// Odometry
std::shared_ptr<lib2131::odometry::BlendedOdometry> DrivenOdom =
    lib2131::odometry::OdometryBuilder::newBuilder()
        .addTrackingWheel(LeftDrivenWheel, lib2131::odometry::WheelLocation::Left)
        .addTrackingWheel(RightDrivenWheel, lib2131::odometry::WheelLocation::Right)
        .addInertialUnit(Inertial)
        .buildBlendedOdometry();

std::shared_ptr<lib2131::odometry::WheelOdometry> DeadOdom =
    lib2131::odometry::OdometryBuilder::newBuilder()
        .addTrackingWheel(LeftDeadWheel, lib2131::odometry::WheelLocation::Left)
        .addTrackingWheel(RightDeadWheel, lib2131::odometry::WheelLocation::Right)
        .addTrackingWheel(RearWheel, lib2131::odometry::WheelLocation::Rear)
        .buildWheelOdometry();

// Controller
pros::Controller primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);