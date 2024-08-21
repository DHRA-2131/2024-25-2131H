#include "main.h"

#include <math.h>

#include "lib2131/utilities/MotionProfile.hpp"
#include "lib2131/utilities/Trajectory.hpp"
#include "lib2131/utilities/Units.h"
#include "main/robot-config.hpp"

using namespace units::literals;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  // DrivenOdom->calibrate();
  // DeadOdom->calibrate();
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
using namespace units::literals;
void opcontrol()
{
  double lastT = 0;
  units::time::second_t OPControlTime = 0_ms;

  lib2131::utilities::Trajectory path({0_in, 0_in, 0_deg}, {100_in, 100_in, 0_deg},
                                      100_in, -100_in, 30);
  lib2131::utilities::MotionProfile<units::length::inches> linearPath(
      path.getPathLength().to<double>(), 100, 40, 40, 0, 0);

  while (true)
  {
    double t = path.solveFromDistance(linearPath.getDistance(OPControlTime), lastT);

    auto linearVel = linearPath.getVelocity(OPControlTime);
    double k = path.getCurvature(t);
    units::angular_velocity::radians_per_second_t angularVel(linearVel.to<double>() * k);

    if (OPControlTime < linearPath.getTotalTime())
    {
      std::cout << t << ", " << k << "," << linearVel << ", " << angularVel << std::endl;
    }
    pros::delay(10);
    OPControlTime += 10_ms;

    lastT = t;
  }
}
