#include "main.h"

#include "2131H/Systems/Chassis/Exit-Conditions/settle-exit-condition.hpp"
#include "2131H/Systems/Chassis/chassis.hpp"
#include "2131H/Systems/Odometry/wheel-odometry.hpp"
#include "2131H/Utilities/console.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

pros::MotorGroup leftDrive({-8, -9, -6}, pros::v5::MotorGearset::blue);
pros::MotorGroup rightDrive({18, 15, 20}, pros::v5::MotorGearset::blue);
pros::IMU inertial(21);

Systems::ChassisParameters chassisInfo{&leftDrive, &rightDrive, 450, 600, 2.75, 13.5};

Systems::TrackingWheel leftTracker(&leftDrive, -7.75, 2.75, 450);
Systems::TrackingWheel rightTracker(&rightDrive, 7.75, 2.75, 450);
Systems::WheelOdometry odometryInfo({&leftTracker, &rightTracker, nullptr, &inertial});

Systems::ExitCondition::Settle lateralExit(1000, 2);
Systems::PID lateralPID(0, 0, 0, {&lateralExit});

Systems::ExitCondition::Settle angularExit(1000, 3);
Systems::PID angularPID(0, 0, 0, {&angularExit});

Systems::Chassis chassis(chassisInfo, &odometryInfo, &lateralPID, &angularPID);

/**
 * @brief Runs at start of program
 *
 */
void initialize()
{
  Console.log(Utilities::Logger::BG_Green, "Initial: ");
  chassis.init();
  Console.log(Utilities::Logger::FG_Green, "  - Chassis Calibration Completed");
  Console.newLine();
}

/**
 * @brief Runs during disabled period. AKA Before Auton, Between Auton and Driver, and at the end of
 * a match
 *
 */
void disabled() { Console.log("Disabled"); }

/**
 * @brief Runs when plugged into Competition Switch or Field Control
 *
 */
void competition_initialize()
{
  Console.log(Utilities::Logger::BG_Green, Utilities::Logger::Underlined, "Competition Inital");
}

/**
 * @brief Runs when Field Control is toggled to (Enable + Autonomous)
 *
 */
void autonomous() { Console.log(Utilities::Logger::BG_Light_Cyan, "Autonomous"); }

/**
 * @brief Runs when Feild Control is toggled to (Enable + Driver Control)
 * @note The name "opcontrol" is short for Teleoperator Control AKA Driver Control
 */
void opcontrol()
{
  Console.log(Utilities::Logger::BG_Light_Blue, "Operator Control: ");

  while (true)
  {
    // chassis.logPosition();
    pros::delay(10);
  }
}