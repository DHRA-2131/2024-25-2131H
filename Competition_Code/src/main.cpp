#include "main.h"

#include "lib2131/api.hpp"

// Motors + MotorGroups
pros::Motor LeftFront(-4, pros::v5::MotorGearset::green);
pros::Motor LeftMid(-1, pros::v5::MotorGearset::green);
pros::Motor LeftRear(-3, pros::v5::MotorGearset::green);

pros::Motor RightFront(5, pros::v5::MotorGearset::green);
pros::Motor RightMid(8, pros::v5::MotorGearset::green);
pros::Motor RightRear(6, pros::v5::MotorGearset::green);

pros::v5::MotorGroup LeftDrive({-4, -1, -2}, pros::v5::MotorGearset::green);
pros::v5::MotorGroup RightDrive({5, 8, 6}, pros::v5::MotorGearset::green);

// Sensors
pros::Rotation RearEncoder(10);
pros::Rotation LeftEncoder(19);
pros::Rotation RightEncoder(-20);

pros::IMU Inertial(21);

// Tracking Wheels
lib2131::TrackingWheel RearWheel(&RearEncoder, 2.0, 5);
lib2131::TrackingWheel LeftDeadWheel(&LeftEncoder, 2.0, -7 / 2);
lib2131::TrackingWheel RightDeadWheel(&RightEncoder, 2.0, 7 / 2);

// Drivetrain T-Wheels
lib2131::TrackingWheel LeftDriveWheel(&LeftDrive, 3.25, -12.75 / 2, 333 + (1 / 3));
lib2131::TrackingWheel RightDriveWheel(&RightDrive, 3.25, 12.75 / 2, 333 + (1 / 3));

// Odometry
lib2131::Odometry TWheelOdom(&LeftDeadWheel, &RightDeadWheel, &RearWheel, nullptr);
lib2131::Odometry DriveOdom(&LeftDriveWheel, nullptr, &RearWheel, &Inertial);

lib2131::PID linearPID(1, 0, 0);
lib2131::PID angularPID(1, 0, 0);

// Robot
lib2131::Robot Robot(9999.0, -9999.0, 56.68, {TWheelOdom, DriveOdom}, angularPID,
                     linearPID, &LeftDrive, &RightDrive);

// Controller
pros::Controller primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);

void Screen()
{
  pros::lcd::initialize();
  while (true)
  {
    Robot.update(10);
    auto position = Robot.getRobotState().position;
    pros::lcd::print(1, "Position:");
    pros::lcd::print(2, "DO: (%f, %f, %f)", position.x, position.y, position.z);
    // pros::lcd::print(2, "DO: (%f, %f, %f)", DriveOdom.getRobotState().position.x,
    //                  DriveOdom.getRobotState().position.y,
    //                  DriveOdom.getRobotState().position.z);
    // pros::lcd::print(3, "TO: (%f, %f, %f)", TWheelOdom.getRobotState().position.x,
    //                  TWheelOdom.getRobotState().position.y,
    //                  TWheelOdom.getRobotState().position.z);
    pros::delay(10);
  }
};

// Screen
pros::Task screenTask(Screen, "SCREEN TASK");

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  Inertial.reset();
  while (Inertial.is_calibrating()) pros::delay(10);
}

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
void autonomous()
{
  std::cout << "=================================\n"
            << "AUTONOMOUS START" << "\n"
            << "=================================" << std::endl;

  Robot.setPosition(lib2131::RobotState({0, 0, lib2131::Angle(0, true)},
                                        {0, 0, lib2131::Angle(0, true)},
                                        {0, 0, lib2131::Angle(0, true)}));

  Robot.moveToPoint(12, 12, false);
  std::cout << "=================================" << "\n";
  std::cout << "AUTONOMOUS END" << std::endl;
  std::cout << "=================================" << "\n";
}

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
  while (true)
  {
    LeftDrive.move_voltage(
        (primary.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y) /
         100) *
        12000);
    RightDrive.move_voltage(
        (primary.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y) /
         100) *
        12000);

    pros::delay(10);
  }
}