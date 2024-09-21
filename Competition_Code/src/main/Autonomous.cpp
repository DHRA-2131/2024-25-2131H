#include "main/Autonomous.hpp"

#include "RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace Autonomous
{
using namespace Systems;

void lowStake(bool isRedTeam)
{
  if (isRedTeam) { chassis.setPose(48, 15.5, -180); }
  else { chassis.setPose(48, 63.5, 0); }

  // Back up to Goal
  chassis.moveToPoint(48, 48, 1000, {false}, false);

  // Clamp, let Goal lift
  Clamp::pneumatic.set_value(1);
  pros::delay(300);

  // Intake::intake preload onto goal
  Intake::motor.move_voltage(12000);

  pros::delay(2000);
  Clamp::pneumatic.set_value(0);
  // // Score more
  // chassis.moveToPoint(74, 24, 1200, {}, false);
  // pros::delay(1000);
  // pneumatic.set_value(0);
  // Intake::intake.brake();
  // chassis.turnToPoint(96, 48, 1000, {false}, false);
  // chassis.moveToPoint(96, 48, 1000, {false}, false);
  // pneumatic.set_value(1);
  // pros::delay(200);
}
void right(bool isRedTeam) {}
void skills(bool isRedTeam) {}
void debug(bool isRedTeam)
{
  chassis.setPose({0, 0, 0});
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(0, 24, 1000);
}
}  // namespace Autonomous