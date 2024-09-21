#include "main/Autonomous.hpp"

#include "RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace Autonomous
{
void lowStake(bool isRedTeam)
{
  if (isRedTeam) { chassis.setPose(48, 15.5, -180); }
  else { chassis.setPose(48, 63.5, 0); }

  // Back up to Goal
  chassis.moveToPoint(48, 48, 1000, {false}, false);

  // Clamp, let Goal lift
  clamp.set_value(1);
  pros::delay(300);

  // Intake preload onto goal
  intake.move_voltage(12000);

  pros::delay(2000);
  clamp.set_value(0);
  // // Score more
  // chassis.moveToPoint(74, 24, 1200, {}, false);
  // pros::delay(1000);
  // clamp.set_value(0);
  // intake.brake();
  // chassis.turnToPoint(96, 48, 1000, {false}, false);
  // chassis.moveToPoint(96, 48, 1000, {false}, false);
  // clamp.set_value(1);
  // pros::delay(200);
}
void right(bool isRedTeam) {}
void skills(bool isRedTeam) {}
void debug(bool isRedTeam)
{
  chassis.setPose({0, 0, 0});
  intake.move_voltage(12000);
  chassis.moveToPoint(0, 24, 1000);
}
}  // namespace Autonomous