#include "main.h"

#include "2131H/Systems/Intake.hpp"
#include "2131H/Systems/Screen.hpp"
#include "Competition/RobotConfig.hpp"
#include "pros/misc.h"

void initialize()
{
  chassis.calibrate(true);
  intake.init();
  Console.useGUI(false);
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
  // intake.enableSort(Screen::isRedTeam() ? Intake::RingColors::BLUE : Intake::RingColors::RED);
  Screen::getAuton()->getAutonCB()(Screen::isRedTeam());
}

void opcontrol()
{
  intake.disableSort();

  intake.drop();
  arm.enable();

  while (true)
  {
    // System TeleOps
    arm.teleOp();
    intake.teleOp();
    clamp.teleOp();
    doinklerRight.teleOp();
    doinklerLeft.teleOp();
    hang.teleOp();

    // Chassis TeleOps
    chassis.tank(primary.get_analog(ANALOG_LEFT_Y), primary.get_analog(ANALOG_RIGHT_Y));

    pros::delay(10);
  }
}