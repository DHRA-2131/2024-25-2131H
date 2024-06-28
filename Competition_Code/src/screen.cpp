/**
 * @file screen.cpp
 * @author Andrew Hilton (2131H)
 * @brief Screen Src Code
 * @version 0.1
 * @date 2024-06-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "main/screen.hpp"

#include "lib2131/utilities/Pose.hpp"
#include "pros/llemu.hpp"

pros::Task ScreenTask([]() {
  pros::lcd::initialize();
  while (true)
  {
    lib2131::utilities::Pose BaseOdom = DrivenOdom->getState().Position;
    pros::lcd::print(1, "Drive Odometry: ");
    pros::lcd::print(2, "{%f, %f, %f}", BaseOdom.pos.x, BaseOdom.pos.y,
                     BaseOdom.theta.convert<units::angle::degrees>());

    lib2131::utilities::Pose WheelOdom = DeadOdom->getState().Position;
    pros::lcd::print(4, "Tracking Wheel Odometry: ");
    pros::lcd::print(5, "{%f, %f, %f}", WheelOdom.pos.x, WheelOdom.pos.y,
                     WheelOdom.theta.convert<units::angle::degrees>());

    pros::lcd::print(6, "ODOM: %f, %f", LeftDeadWheel->getDistance(),
                     RightDeadWheel->getDistance());
    pros::lcd::print(7, "DRIVE: %f, %f", LeftDrivenWheel->getDistance(),
                     RightDrivenWheel->getDistance());

    pros::delay(RefreshRate);
  }
});