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

#include "pros/llemu.hpp"

pros::Task ScreenTask([]() {
  pros::lcd::initialize();
  while (true)
  {
    auto driveOdomPosition = DriveOdom.getRobotState().getPosition();
    pros::lcd::print(1, "Drive Odometry: ");
    pros::lcd::print(2, "{%f, %f, %f}", driveOdomPosition->x, driveOdomPosition->y,
                     driveOdomPosition->z.getDegrees());

    auto tWheelOdom = TWheelOdom.getRobotState().getPosition();
    pros::lcd::print(4, "Tracking Wheel Odometry: ");
    pros::lcd::print(5, "{%f, %f, %f}", tWheelOdom->x, tWheelOdom->y,
                     tWheelOdom->z.getDegrees());
    pros::lcd::print(6, "TOTW: %f, %f", LeftDeadWheel.getDistance(),
                     RightDeadWheel.getDistance());
    pros::lcd::print(7, "DOTW: %f, %f", LeftDriveWheel.getDistance(),
                     RightDriveWheel.getDistance());

    pros::delay(RefreshRate);
  }
});