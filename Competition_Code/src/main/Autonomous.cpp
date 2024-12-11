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
    //chassis.moveToPoint(0,  0, 12000, {false, 127, 0}, false,);
    // chassis.turnToPoint(0, 0, 12000, {true, AngularDirection::CCW_COUNTERCLOCKWISE, 127, 0}, false);

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
    chassis.moveToPoint(51, 55, 1000, {.forwards = true}, false);
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

void highStake(bool isRedTeam)
{
  if (isRedTeam) {}
  else {}
}

void goalRush(bool isRedTeam)
{
  if (isRedTeam) {
    //Intake::enableAutoSort();
    Arm::setPosition(0);
    Clamp::enableAutoClamp();
    chassis.setPose({-11.55, 19, 180});
    chassis.moveToPoint(-14.5, 48, 1000, {.forwards = false}); // move by ring stakc
    chassis.turnToPoint(-26, 70, 50, {.forwards = false}); //turn to goal
    chassis.moveToPoint(-19.6, 63, 1000, {.forwards = false}); //move to goal
    pros::c::delay(650); //clamp goal
    Clamp::pneumatic.extend();
    chassis.turnToPoint(-25, 48, 500, {.forwards = true}); //turn to ring stack
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(-24, 47, 1000); //move to ring stack
    chassis.turnToPoint(0, 30, 800, {.forwards = false}, true); //turn goal towards corner
    pros::c::delay(900);
      Clamp::disableAutoClamp(); //drop goal
    chassis.turnToPoint(-48, 44, 800, {.forwards = false}, false); //turn towards ladder goal
    Clamp::pneumatic.retract();
    Intake::intake.move_voltage(0);
    chassis.moveToPoint(-38, 48, 1000, {.forwards = false}, true); //move to goal
    Clamp::enableAutoClamp();
    pros::c::delay(1000); //clamp goal
    chassis.turnToPoint(-42, 0, 800); //turn to driver wall
    Clamp::pneumatic.extend();
    chassis.moveToPoint(-50, 17, 1000); //move towards wall
    Intake::intake.move_voltage(12000);
    chassis.turnToPoint(0, 6, 800); //turn to preload
    chassis.moveToPoint(-29, 16, 1000); //move to preload
    pros::c::delay(1500); //score preload
    chassis.moveToPoint(-4, 10, 1000); //move to corner
  }

  else
  {
    Intake::enableAutoSort();
    Arm::setPosition(2);
    Clamp::enableAutoClamp();
    chassis.setPose({11.75, 20, 0});
    chassis.moveToPoint(15, 48, 1000, {.forwards = true}, true);   // move to goal
    chassis.moveToPoint(20.1, 59.25, 1000, {.forwards = true}, false);  // move to goal
    // chassis.moveToPoint(19, 56.1, 1000, {.forwards = true}, false); //move to goal
    // chassis.moveToPoint(17.25, 58, 1000, {.forwards = true}, false); //move to goal
    // chassis.turnToPoint(26, 72, 450, {}, false); //turn to goal
    Arm::setPosition(3);  // score preload
    pros::c::delay(250);
    chassis.turnToPoint(60, 55, 1000, {}, true);                    // tip goal
    pros::c::delay(80);                                            // lower, less tip, higher, more tip but getting stuck
    Arm::setPosition(4);                                            // remove 3rd goal
    chassis.moveToPoint(18, 38, 1000, {.forwards = false}, false);  // move away from line
    Arm::setPosition(0);                                            // lift arm away from intake
    pros::c::delay(100);
    Intake::intake.move_voltage(12000);
    chassis.moveToPoint(23, 53, 1000, {.forwards = true}, false);  // intake 1st ring
    pros::c::delay(80);                                           // intake 1st ring
    Intake::intake.move_voltage(0);
    chassis.turnToPoint(55, 0, 1000, {.forwards = false}, false);  // turn to goal
    chassis.moveToPoint(27.5, 40, 1000, {.forwards = false});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false});
    chassis.moveToPoint(43, 48, 1000, {.forwards = false}, false);  // move to goal
    Intake::intake.move_voltage(12000);                              // score ring
    pros::c::delay(300);                                           // score 1st goal
    chassis.turnToPoint(72, 24, 1000, {}, false); //turn to inverse stack
    chassis.moveToPoint(60, 33, 1000, {.forwards = true}, false); //move to inverse stack
    Clamp::disableAutoClamp();
    chassis.moveToPoint(100,  0, 1000, {true, 50, 0}, false); //move through both rings
    chassis.moveToPoint(72, 24, 1000, {.forwards = false}, true); //back up in front of the alliance stake
    pros::c::delay(100);
    Arm::setPosition(1);
    chassis.turnToPoint(72, 0, 1000);
    pros::c::delay(500);
    Intake::intake.move_voltage(0);
    chassis.moveToPoint(68.5, 16.5, 1000);
    Arm::setPosition(3);
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