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

void lowAlliance(bool isRedTeam)
{
  if (isRedTeam)
  {
    //chassis.moveToPoint(0,  0, 12000, {false, 127, 0}, false);
     //chassis.turnToPoint(0, 0, 12000, {true, AngularDirection::CCW_COUNTERCLOCKWISE, 127, 0}, false);

    // chassis.swingToPoint(0, 0, DriveSide::LEFT, 1000, {false, }, false)
    // Intake::motor.move_voltage(12000);
    // Arm::setPosition(2);
    // Intake::disableAutoSort();
    // Intake::enableAutoSort();

    // Clamp::enableAutoClamp();
    // Clamp::disableAutoClamp();
    Arm::setPosition(1); //1 for alliance stake, 0 for no
    Intake::enableAutoSort();
    Clamp::enableAutoClamp();
    chassis.setPose({54, 16.5, 90});
    chassis.turnToPoint(71.8, 0, 1000, {}, false);
    Arm::setPosition(3); //alliance stake
    chassis.moveToPoint(58.2 , 9.5, 1000, {.forwards = true}, false); //alliance stake
    pros::c::delay(600);  // gets alliance stake
    Arm::setPosition(0); //alliance stake
    chassis.moveToPoint(47, 39.5, 1000, {.forwards = false}, false);
    pros::c::delay(500);
      //Intake::motor.move_voltage(12000); //no alliance stake
    chassis.turnToPoint(0, 48, 1000, {}, false);
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(18, 50, 1000, {.forwards = true}, false);
    pros::c::delay(600);  // 1st ring
    chassis.turnToPoint(14, 68, 1000, {}, false);
    chassis.moveToPoint(14, 57, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 2nd ring
    chassis.moveToPoint(19, 53, 1000, {.forwards = false}, false);
    chassis.turnToPoint(22, 65, 1000, {}, false);
    chassis.moveToPoint(21.5, 59.75, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 3rd ring
    chassis.moveToPoint(22, 40, 1000, {.forwards = false}, false);
    chassis.turnToPoint(72, 72, 1000, {}, false);
    chassis.moveToPoint(51.5, 56, 1000, {.forwards = true}, false);
  }
  else
  {
    Arm::setPosition(1); //1 for alliance stake, 0 for none
    Clamp::enableAutoClamp();
    chassis.setPose({54, 144-16.5, 90});
    chassis.turnToPoint(70.5, 0 + 144, 1000, {}, false);
    Arm::setPosition(3); //0 for no alliance stake, 3 for alliance stake
    chassis.moveToPoint(57, 134, 1000, {.forwards = true}, false); //alliance stake
    pros::c::delay(500);  // alliance ring
    Arm::setPosition(0);
    chassis.moveToPoint(45, 144 - 40.5, 1000, {.forwards = false}, false);
    pros::c::delay(500);
    //Intake::motor.move_voltage(12000); //no alliance stake
    chassis.turnToPoint(0, 94, 1000, {}, false);
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(16.5, 90.8, 1000, {.forwards = true}, false);
    pros::c::delay(600);  // 1st ring
    chassis.turnToPoint(14, 144 - 68, 1000, {}, false);
    chassis.moveToPoint(17, 83, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 2nd ring
    chassis.moveToPoint(20, 144 - 53, 1000, {.forwards = false}, false);
    chassis.turnToPoint(22, 144 - 65, 1000, {}, false);
    chassis.moveToPoint(21.5, 81.5, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 3rd ring
    chassis.moveToPoint(22, 144 - 40, 1000, {.forwards = false}, false);
    chassis.turnToPoint(72, 144 - 72, 1000, {}, false);
    chassis.moveToPoint(55.5, 87, 2000, {.forwards = true}, false);

  }
}
void low4RG(bool isRedTeam)
{
  if (isRedTeam) {
    Arm::setPosition(0); //1 for alliance stake, 0 for no
    Intake::enableAutoSort();
    Clamp::enableAutoClamp();
    chassis.setPose({54, 16.5, 90});
    chassis.turnToPoint(71.8, 0, 1000, {}, false);
    //Arm::setPosition(3); //alliance stake
    //chassis.moveToPoint(58.2 , 9.5, 1000, {.forwards = true}, false); //alliance stake
    pros::c::delay(600);  // gets alliance stake
    //Arm::setPosition(0); //alliance stake
    chassis.moveToPoint(47, 39.5, 1000, {.forwards = false}, false);
    pros::c::delay(500);
      Intake::intake.move_voltage(12000); //no alliance stake
    chassis.turnToPoint(0, 48, 1000, {}, false);
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(18, 50, 1000, {.forwards = true}, false);
    pros::c::delay(600);  // 1st ring
    chassis.turnToPoint(14, 68, 1000, {}, false);
    chassis.moveToPoint(14, 57, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 2nd ring
    chassis.moveToPoint(19, 53, 1000, {.forwards = false}, false);
    chassis.turnToPoint(22, 65, 1000, {}, false);
    chassis.moveToPoint(21.5, 59.75, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 3rd ring
    chassis.moveToPoint(22, 40, 1000, {.forwards = false}, false);
    chassis.turnToPoint(72, 72, 1000, {}, false);
    chassis.moveToPoint(51, 55, 1000, {.forwards = true}, false);
  }
  else {
    Arm::setPosition(0); //1 for alliance stake, 0 for none
    Clamp::enableAutoClamp();
    chassis.setPose({54, 144-16.5, 90});
    chassis.turnToPoint(70.5, 0 + 144, 1000, {}, false);
    //Arm::setPosition(3); //0 for no alliance stake, 3 for alliance stake
    //chassis.moveToPoint(57, 134, 1000, {.forwards = true}, false); //alliance stake
    pros::c::delay(500);  // alliance ring
    Arm::setPosition(0);
    chassis.moveToPoint(45, 144 - 40.5, 1000, {.forwards = false}, false);
    pros::c::delay(500);
    Intake::intake.move_voltage(12000); //no alliance stake
    chassis.turnToPoint(0, 94, 1000, {}, false);
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(16.5, 90.8, 1000, {.forwards = true}, false);
    pros::c::delay(600);  // 1st ring
    chassis.turnToPoint(14, 144 - 68, 1000, {}, false);
    chassis.moveToPoint(17, 83, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 2nd ring
    chassis.moveToPoint(20, 144 - 53, 1000, {.forwards = false}, false);
    chassis.turnToPoint(22, 144 - 65, 1000, {}, false);
    chassis.moveToPoint(21.5, 81.5, 1000, {.forwards = true}, false);
    pros::c::delay(550);  // 3rd ring
    chassis.moveToPoint(22, 144 - 40, 1000, {.forwards = false}, false);
    chassis.turnToPoint(72, 144 - 72, 1000, {}, false);
    chassis.moveToPoint(55.5, 87, 2000, {.forwards = true}, false);
  }
}

void highStake(bool isRedTeam) //not high stake, actually low stake 5 ring with alliance
{
  if (isRedTeam) {
     Arm::setPosition(1); //1 for alliance stake, 0 for no
    Clamp::pneumatic.retract();
    Intake::enableAutoSort();
    Clamp::enableAutoClamp();
    chassis.setPose({54, 16.5, 90});
    chassis.turnToPoint(72.3, 0, 500, {}, false); //turn to alliance stake
    Arm::setPosition(3); //alliance stake
    chassis.moveToPoint(58.2 , 9.5, 1000, {.forwards = true}, false); //alliance stake
    pros::c::delay(500);  // gets alliance stake
    chassis.moveToPoint(49, 39.8, 1000, {.forwards = false}, false); //back up to mogo
    pros::c::delay(300); //clamp goal
    Clamp::pneumatic.extend();
    chassis.turnToPoint(72, 24, 500); //turn to reverse stack
    chassis.moveToPoint(59.5, 40, 1000, {.forwards = true}, true); //move to reverse stack
    Arm::setPosition(0); //alliance stake
    pros::c::delay(550);  //doink ring
    Arm::doinkler.extend();
    pros::c::delay(300);  //doink ring
    chassis.moveToPoint(36, 26, 900, {.forwards = false}); //back ring off reverse stack
    chassis.turnToPoint(12, 48, 500, {}, false); //turn to ring to single stack
    Arm::doinkler.retract();
    Intake::intake.move_voltage(12000);
    chassis.turnToPoint(24, 72, 300); //turn to 2 rings
    chassis.moveToPoint(13, 48, 1000, {.forwards = true}, false); //intake 2 rings
    pros::c::delay(100);  //intake 2 rings
    chassis.turnToPoint(44, 59, 500, {.forwards = false});
    chassis.moveToPoint(44, 56, 1000, {.forwards = false}); //back up to line up to to ring pile
    chassis.turnToPoint(26, 68, 500); //turn to 1st ring in ring pile
    chassis.moveToPoint(30, 58, 900, {.forwards = true}, false); //move to ring pile
    chassis.moveToPoint(23, 56, 1000); //move to 2nd ring in pile
    chassis.moveToPoint(36, 48, 1000, {.forwards = false}); //back away from line
    chassis.turnToPoint(-42, 0, 500); //turn to corner
    chassis.moveToPoint(12, 12, 1000); //move to corner fast
    chassis.moveToPoint(5,  5, 1000, {true, 50, 0}, false);//move to corner slow
    chassis.moveToPoint(58,  58, 1000, {false, 80, 0}, false); //back out of the corner
    

    
  }
  else {

  }

  }


void goalRush(bool isRedTeam)
{
  if (isRedTeam) {
  Intake::enableAutoSort();
  Arm::setPosition(0);
  Clamp::enableAutoClamp();
  chassis.setPose({-12, 19, 180});
    chassis.moveToPoint(-17.5, 52, 10000, {.forwards = false}); // move by ring stack
    chassis.turnToPoint(-25, 70, 50, {.forwards = false}); //turn to goal
    chassis.moveToPoint(-20.5, 64.5, 1000, {.forwards = false}); //move to goal
    pros::c::delay(600); //grab goal
    chassis.turnToPoint(-25, 48, 500, {.forwards = true}); //turn to ring stack
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(-30.2, 51, 1000); //move to ring stack
    chassis.turnToPoint(0, 30, 800, {.forwards = false}, false); //turn goal towards corner
      Clamp::disableAutoClamp(); //drop goal
    chassis.turnToPoint(-48, 53, 800, {false, AngularDirection::CCW_COUNTERCLOCKWISE, 127, 0}, false); //turn towards ladder goal
    Intake::intake.move_voltage(0);
    chassis.moveToPoint(-42, 48, 1000, {.forwards = false}); //move to goal
    Clamp::enableAutoClamp();
    pros::c::delay(400); //grab goal
    Clamp::pneumatic.extend();
    chassis.turnToPoint(-48, 0, 800); //turn to driver wall
    chassis.moveToPoint(-52, 17, 1000); //move towards wall
    Intake::intake.move_voltage(12000);
    chassis.turnToPoint(0, 6, 800); //turn to preload
    chassis.moveToPoint(-14, 17, 1000); //move to preload
    pros::c::delay(1000); //score preload
    chassis.turnToPoint(0, 5, 1000); //turn to corner
    chassis.moveToPoint(-1.5,  6.5, 1000, {true, 50, 0}, false); //move to corner
     pros::c::delay(500); //score corner ring
     chassis.moveToPoint(-24,  24, 1000, {false}, false); //back up from corner
      //chassis.turnToPoint(-72, 72, 1000); // turn to ladder
      //chassis.moveToPoint(-52, 53.8, 1000); //move to ladder
  }

  else
  {
       Intake::enableAutoSort();
    Arm::setPosition(0);
    Clamp::enableAutoClamp();
    chassis.setPose({-12, 144-19, 0});
    chassis.moveToPoint(-16.25, 94, 10000, {.forwards = false}); // move by ring stack
    chassis.turnToPoint(-26, 144-70, 50, {.forwards = false}); //turn to goal
    chassis.moveToPoint(-21, 81.3, 1000, {.forwards = false}); //move to goal
    pros::c::delay(700); //grab goal
    Clamp::pneumatic.extend();
    pros::c::delay(100); //grab goal
    chassis.turnToPoint(-25, 95, 500, {.forwards = true}); //turn to ring stack
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(-29.8, 91, 1000); //move to ring stack
    chassis.turnToPoint(0, 144-30, 800, {.forwards = false}, false); //turn goal towards corner
      Clamp::disableAutoClamp(); //drop goal
    chassis.turnToPoint(-48, 85, 800, {false, AngularDirection::CCW_COUNTERCLOCKWISE, 127, 0}, false); //turn towards ladder goal
    Intake::intake.move_voltage(0);
    chassis.moveToPoint(-42.5, 87, 1000, {.forwards = false}); //move to goal
    Clamp::enableAutoClamp();
    pros::c::delay(500); //grab goal
    Clamp::pneumatic.extend();
    chassis.turnToPoint(-60, 144-0, 800); //turn to driver wall
    chassis.moveToPoint(-60, 120, 1000); //move towards wall
    Intake::intake.move_voltage(12000);
    chassis.turnToPoint(0, 144-6, 800); //turn to preload
    chassis.moveToPoint(-27, 128, 1000); //move to preload
    pros::c::delay(1000); //score preload
    chassis.turnToPoint(.5, 144, 1000); //turn to corner
    chassis.moveToPoint(-2,  144-6, 1000, {true, 50, 0}, false); //move to corner
     pros::c::delay(100); //score corner ring
     chassis.moveToPoint(-24,  144-24, 1000, {false}, false); //back up from corner
     //pros::c::delay(500); //score corner ring
     // chassis.turnToPoint(-72, 144-72, 1000); // turn to ladder
//Intake::intake.move_voltage(0);
      //chassis.moveToPoint(-53, 82, 1000); //move to ladder
    //Arm::doinkler.extend();
    //pros::c::delay( 300);
    //chassis.moveToPoint(50, 32, 1000, {.forwards = false}, false);
    //Arm::doinkler.retract();
  }
}
void skills(bool isRedTeam) {}

void debug(bool isRedTeam) { chassis.setPose({0, 0, 0}); }

}  // namespace Autonomous
// namespace Autonomous