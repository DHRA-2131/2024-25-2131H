#include <sys/types.h>

#include <cmath>

#include "2131H/Systems/Intake.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

namespace Autonomous
{
// Solo Win Point
void soloAWP(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.setPose({48 + 7.00, 24 - 7.75 - 2, 90});
    chassis.turnToPoint(71.5, 0.2, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30});
    chassis.turnToPoint(48, 44 - 2, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(48, 44 - 2, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1
    intake.spin();
    chassis.turnToPoint(32, 42, 1000, {.minSpeed = 50});
    chassis.moveToPoint(32, 42, 2000, {.minSpeed = 30}, false);
    pros::delay(200);

    // ? Ring 2
    chassis.moveToPoint(45.3, 24 - 3, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToPoint(60.5, 24 - 3, 1000, {.minSpeed = 30});
    intake.lift();
    chassis.moveToPoint(60.5, 24 - 3, 2000, {.maxSpeed = 80}, false);
    intake.drop();
    chassis.moveLinear(-9, 2000, {}, false);
    pros::delay(500);

    clamp.disableAutoClamp();

    // * Goal 2
    chassis.moveToPoint(96, 26, 1000);
    intake.stop();
    chassis.turnToPoint(96, 46, 1000, {.forwards = false, .minSpeed = 20}, false);
    clamp.enableAutoClamp();
    intake.spin(-12000);
    arm.setIndex(0);
    chassis.moveToPoint(90, 46, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 20}, false);

    // ? Ring 1
    intake.disableSort();
    intake.spin();
    chassis.turnToHeading(90, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(20, 2000, {}, false);

    // * Ladder
    chassis.turnToHeading(100, 2000, {.minSpeed = 30}, false);
    pros::delay(2000);
    chassis.moveLinear(-38, 2000, {});
    pros::delay(600);
    intake.stop();
  }
  else { soloAWP(true); }
}

// Safe Autonomous
void safeRing(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam) {}
  else {}
}

void safeGoal(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam) {}
  else {}
}

// Four Ring Autos
void fourRingGoal(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    // pros::delay(2000);
    chassis.setPose({-48 - 7.00 - 2, 24 - 7.75 + 1, -90});
    chassis.turnToPoint(-72, 3.25, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(7.5, 1000, {});
    arm.setPosition(210);

    // * Goal 1
    chassis.moveLinear(-14, 2000, {.minSpeed = 30});
    chassis.turnToPoint(-48, 48, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(-48, 48, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1
    intake.lift();
    intake.spin();
    chassis.moveToPoint(-63, 27, 2500, {.minSpeed = 10}, false);
    intake.drop();
    chassis.moveLinear(-12, 1000, {.forwards = false, .minSpeed = 30});

    // ? Ring 2
    pros::Motor firstStage(-14);
    intake.stop();
    firstStage.move_voltage(12000);
    chassis.turnToPoint(-24, 24, 1000, {.minSpeed = 4}, false);
    intake.spin();
    chassis.moveToPoint(-24, 50, 2000, {}, false);
    intake.stop();
    firstStage.move_voltage(12000);

    // * To Corner
    chassis.turnToHeading(-5, 1000, {.minSpeed = 40}, false);
    intake.spin();
    chassis.movePolar(-30, -5, 2000, {}, false);
    pros::delay(500);
    arm.setIndex(0);
    chassis.turnToHeading(135, 2000, {.minSpeed = 20}, false);
    firstStage.move_voltage(-12000);
    chassis.movePolar(25.5, 135, 1000, {.maxSpeed = 50}, false, false);
    intake.spin();
    pros::delay(350);

    // ? Ring 4
    chassis.moveLinear(-4, 2000, {}, false);
    intake.lift();
    intake.spin();
    chassis.moveLinear(5, 1000, {.minSpeed = 20});
    firstStage.move_voltage(-12000);
    pros::delay(200);
    intake.spin();
    pros::delay(200);
    chassis.movePolar(-12, 135, 1000, {.minSpeed = 20}, false, false);
    intake.drop();

    // * Ladder
    chassis.turnToHeading(-45, 1000, {.minSpeed = 20}, false);
    chassis.movePolar(45, -45, 2000, {.maxSpeed = 127}, false);
  }
  else { fourRingRing(true); }
}

void fourRingRing(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    //  pros::delay(2000);
    chassis.setPose({48 + 7.00, 24 - 7.75, 90});
    chassis.turnToPoint(72, 3.25, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(7.75, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-14, 2000, {.minSpeed = 30});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false, .minSpeed = 40});
    clamp.enableAutoClamp();
    chassis.moveToPoint(48, 48, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1
    intake.lift();
    intake.spin();
    chassis.moveToPoint(67, 28, 2500, {.minSpeed = 10}, false);
    intake.drop();
    chassis.moveLinear(-12, 1000, {.forwards = false, .minSpeed = 30});

    // ? Ring 2
    pros::Motor firstStage(-14);
    intake.stop();
    firstStage.move_voltage(12000);
    chassis.turnToPoint(28, 24, 1000, {.minSpeed = 20}, false);
    intake.spin();
    chassis.moveToPoint(28, 50, 2000, {}, false);
    intake.stop();
    firstStage.move_voltage(12000);

    // * To Corner
    chassis.turnToHeading(2, 1000, {.minSpeed = 40}, false);
    intake.spin();
    chassis.movePolar(-33, 2, 2000, {}, false);
    pros::delay(500);
    arm.setIndex(0);
    chassis.turnToHeading(-135, 2000, {.minSpeed = 20}, false);
    firstStage.move_voltage(-12000);
    chassis.movePolar(25.5, -135, 1000, {.maxSpeed = 50}, false, false);
    intake.spin();
    pros::delay(350);

    // ? Ring 4
    chassis.moveLinear(-4, 2000, {}, false);
    intake.lift();
    intake.spin();
    chassis.moveLinear(5, 1000, {.minSpeed = 20});
    firstStage.move_voltage(-12000);
    pros::delay(200);
    intake.spin();
    pros::delay(200);
    chassis.movePolar(-12, -135, 1000, {.minSpeed = 20}, false, false);
    intake.drop();

    // * Ladder
    chassis.turnToHeading(45, 1000, {.minSpeed = 20}, false);
    chassis.movePolar(45, 45, 2000, {.maxSpeed = 127}, false);
  }
  else { fourRingGoal(true); }
}

void debug(bool isRedTeam) { chassis.attemptReckonToGoal({72, 120, 0}, &clamp, 1000000); }

}  // namespace Autonomous