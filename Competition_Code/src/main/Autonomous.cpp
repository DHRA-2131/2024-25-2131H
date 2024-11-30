#include "main/Autonomous.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "main/RobotConfig.hpp"
#include "pros/rtos.h"
#include "systems/Arm.hpp"
#include "systems/Clamp.hpp"
#include "systems/Intake.hpp"

namespace Autonomous
{
using namespace Systems;

void lowStake(bool isRedTeam)
{
  if (isRedTeam)
  {
    // chassis.moveToPoint(0,  0, 12000, {false, 127, 0}, false);
    // chassis.turnToPoint(0, 0, 12000, {true, AngularDirection::CCW_COUNTERCLOCKWISE, 127, 0}, false);

    // chassis.swingToPoint(0, 0, DriveSide::LEFT, 1000, {false, }, false)
    // Intake::motor.move_voltage(12000);
    // Arm::setPosition(2);
    // Intake::disableAutoSort();
    // Intake::enableAutoSort();

    // Clamp::enableAutoClamp();
    // Clamp::disableAutoClamp();
    Arm::setPosition(1);
    Clamp::enableAutoClamp();
    chassis.setPose({53, 17, 90});
    chassis.turnToPoint(72, 0, 1000, {}, false);
    Arm::setPosition(3);
    chassis.moveToPoint(58, 13, 1000, {.forwards = true}, false);
     pros::c::delay(250);
     Arm::setPosition(0);
    chassis.moveToPoint(51, 36, 1000, {.forwards = false}, false);
  }
  else {}
}
void soloWP(bool isRedTeam)
{
  if (isRedTeam) {}
  else {}
}

void highStake(bool isRedTeam)
{
  if (isRedTeam) {}
  else {}
}

void skills(bool isRedTeam) {}

void debug(bool isRedTeam) { chassis.setPose({0, 0, 0}); }

}  // namespace Autonomous
// namespace Autonomous