#include "main.h"

#include "lib2131/api.hpp"

// Motors + MotorGroups
pros::Motor LeftFront(-4, pros::v5::MotorGearset::green);
pros::Motor LeftMid(-1, pros::v5::MotorGearset::green);
pros::Motor LeftRear(-3, pros::v5::MotorGearset::green);

pros::Motor RightFront(5, pros::v5::MotorGearset::green);
pros::Motor RightMid(8, pros::v5::MotorGearset::green);
pros::Motor RightRear(6, pros::v5::MotorGearset::green);

pros::v5::MotorGroup LeftDrive({-4, -1, -3}, pros::v5::MotorGearset::green);
pros::v5::MotorGroup RightDrive({5, 8, 6}, pros::v5::MotorGearset::green);

// Sensors
pros::Rotation RearEncoder(-10);
pros::Rotation LeftEncoder(-19);
pros::Rotation RightEncoder(20);

pros::IMU Inertial(21);

// Tracking Wheels
lib2131::trackingWheel RearWheel(&RearEncoder, 2.0, 5);
lib2131::trackingWheel LeftDeadWheel(&LeftEncoder, 2.0, -7 / 2);
lib2131::trackingWheel RightDeadWheel(&RightEncoder, 2.0, 7 / 2);

// Drivetrain T-Wheels
lib2131::trackingWheel LeftDriveWheel(&LeftDrive, 3.25, -12.75 / 2, 333);
lib2131::trackingWheel RightDriveWheel(&RightDrive, 3.25, 12.75 / 2, 333);

// Odometry
lib2131::odometry TWheelOdom(&LeftDeadWheel, &RightDeadWheel, &RearWheel, nullptr);
lib2131::odometry DriveOdom(&LeftDriveWheel, nullptr, &RearWheel, &Inertial);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
  Inertial.reset();
  while (Inertial.is_calibrating()) pros::delay(10);

  while (true)
  {
    DriveOdom.update(10);
    TWheelOdom.update(10);

    // std::cout << LeftDriveWheel.getDistanceTraveled() << ", "
    //           << RightDriveWheel.getDistanceTraveled() << "\n";

    std::cout << DriveOdom.getRobotState().position << ", " << Inertial.get_heading()
              << "\n";
    pros::delay(10);
  }
}