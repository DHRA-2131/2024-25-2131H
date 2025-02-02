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
    chassis.setPose({48 + 7.00, 24 - 7.75, 90});
    chassis.turnToPoint(72, 1.5, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8.5, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(48, 48, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1
    intake.spin();
    chassis.turnToPoint(27, 46, 1000, {.minSpeed = 30});
    chassis.moveToPoint(27, 46, 2000, {.minSpeed = 20}, false);
    pros::delay(200);

    // ? Ring 2
    chassis.moveToPoint(45.3, 24, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToPoint(64, 26, 1000, {.minSpeed = 30});
    intake.lift();
    chassis.moveToPoint(64, 26, 2000, {.maxSpeed = 80}, false);
    pros::delay(1800);
    clamp.disableAutoClamp();

    // * Goal 2
    chassis.moveToPoint(86, 26, 1000);
    intake.drop();
    intake.stop();
    chassis.turnToPoint(90, 50, 1000, {.forwards = false}, false);
    clamp.enableAutoClamp();
    intake.spin(-12000);
    arm.setIndex(0);
    chassis.moveToPoint(90, 50, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 20}, false);

    // ? Ring 1
    intake.disableSort();
    intake.spin();
    chassis.turnToHeading(90, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(19, 2000, {.minSpeed = 10}, false);
    pros::delay(200);
    chassis.turnToHeading(100, 2000, {.minSpeed = 30}, false);
    chassis.moveLinear(-40, 2000, {});

    // * Ladder
    pros::delay(600);
    intake.stop();
  }
  else
  {
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.setPose({48 + 7.00, 24 - 7.75, 90});
    chassis.turnToPoint(72, 2, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8.5, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(48, 48, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1
    intake.spin();
    chassis.turnToPoint(27, 46, 1000, {.minSpeed = 30});
    chassis.moveToPoint(27 - 3, 44, 2000, {.minSpeed = 20}, false);
    pros::delay(100);
    chassis.moveToPoint(27 - 10, 46, 1000, {.forwards = false, .minSpeed = 20});

    // ? Ring 2
    chassis.moveToPoint(45.3, 24, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToPoint(64, 26, 1000, {.minSpeed = 30});
    intake.lift();
    chassis.moveToPoint(64 - 5, 24, 2000, {.maxSpeed = 80}, false);
    pros::delay(1800);
    // ntake.drop();
    // hassis.moveLinear(-5, 2000);
    clamp.disableAutoClamp();

    // * Goal 2
    intake.lift();
    chassis.moveToPoint(86 + 1, 26, 1000);
    intake.drop();
    intake.stop();
    chassis.turnToPoint(90, 50, 1000, {.forwards = false}, false);
    clamp.enableAutoClamp();
    intake.spin(-12000);
    arm.setIndex(0);
    chassis.moveToPoint(
        90, 50 - 3, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 20}, false);

    // ? Ring 1
    intake.disableSort();
    intake.spin();
    chassis.turnToHeading(90, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(19, 2000, {.minSpeed = 10}, false);
    pros::delay(200);
    chassis.turnToHeading(100, 2000, {.minSpeed = 30}, false);
    chassis.moveLinear(-40 + 5, 2000, {.maxSpeed = 99}, false);

    // * Ladder
    pros::delay(600);
    intake.stop();
  }
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
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.setPose({-48 - 7.00, 24 - 7.75, -90});
    chassis.turnToPoint(-72.0, 0, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-18, 2000, {.minSpeed = 30});
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
    // chassis.turnToPoint(24, 52, 1000, {.minSpeed = 20}, false);
    chassis.moveToPoint(-24, 52 - 4, 2000, {}, false);
    pros::delay(200);

    //* Corner
    chassis.turnToPoint(-13, 0, 1000, {.maxSpeed = 60, .minSpeed = 20}, false);
    pros::delay(200);
    arm.setIndex(0);
    intake.spin(-6000);
    chassis.moveToPoint(-13, 19 - 2.5, 2000, {.maxSpeed = 80, .minSpeed = 40});

    // ? Ring 3
    chassis.swingToHeading(130, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(5 + 2.00, 500, {.maxSpeed = 50}, false);
    intake.spin();
    // ros::delay(4000);
    chassis.moveLinear(-3.3 + 1, 1000, {.maxSpeed = 20}, false);
    pros::delay(1300);

    // ? Ring 4
    // intake.lift();
    // intake.spin(-6000);
    // chassis.moveLinear(6, 500, {.maxSpeed = 60}, false);
    // intake.spin();
    // pros::delay(800);
    // chassis.moveLinear(-15, 1000, {.maxSpeed = 30});
    // pros::delay(300);
    // intake.drop();

    // * Ladder
    chassis.turnToHeading(-50 + 2.5, 1000, {.minSpeed = 20}, false);
    intake.stop();
    chassis.moveToPoint(-56, 46, 2000, {.maxSpeed = 75}, false);
    // hassis.moveLinear(48, 2000, {});
  }
  else
  {
    intake.enableSort(Intake::RingColors::RED);
    chassis.setPose({48 + 7.00, 24 - 7.75, 90});
    chassis.turnToPoint(72, 1.5 + 1.2, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8.5, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(48, 48, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1
    intake.lift();
    intake.spin();
    chassis.moveToPoint(68 - 1.5, 28, 2500, {.minSpeed = 10}, false);
    intake.drop();
    chassis.moveLinear(-12, 1000, {.forwards = false, .minSpeed = 30});

    // ? Ring 2
    // chassis.turnToPoint(24, 52, 1000, {.minSpeed = 20}, false);
    chassis.moveToPoint(24 - 6.5, 52 - 2, 2000, {}, false);

    //* Corner
    chassis.turnToPoint(12 - 8.5, 0, 1000, {.minSpeed = 20}, false);
    pros::delay(200);
    arm.setIndex(0);
    intake.spin(-6000);
    chassis.moveToPoint(15 - 3, 17 - 3.2, 2000, {.maxSpeed = 80, .minSpeed = 40});

    // ? Ring 3
    chassis.swingToHeading(-135, lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(5, 500, {.maxSpeed = 60}, false);
    intake.spin();
    pros::delay(400);
    chassis.moveLinear(-3.0, 1000, {.maxSpeed = 30}, false);
    pros::delay(800);

    // ? Ring 4
    intake.lift();
    intake.spin(-6000);
    chassis.moveLinear(6, 500, {.maxSpeed = 60}, false);
    intake.spin();
    pros::delay(800);
    chassis.moveLinear(-15, 1000, {.maxSpeed = 30});
    pros::delay(300);
    intake.drop();

    // * Ladder
    chassis.turnToHeading(48 - 2, 1000, {.minSpeed = 20}, false);
    intake.stop();
    chassis.moveLinear(47, 2000, {});
  }
}

void fourRingRing(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    //  pros::delay(2000);
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.setPose({48 + 7.00, 24 - 7.75, 90});
    chassis.turnToPoint(72, 1.5, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8.5, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false, .minSpeed = 30});
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
    // chassis.turnToPoint(24, 52, 1000, {.minSpeed = 20}, false);
    chassis.moveToPoint(23, 50, 2000, {}, false);

    //* Corner
    chassis.turnToPoint(10, 0, 1000, {.minSpeed = 20}, false);
    pros::delay(200);
    arm.setIndex(0);
    intake.spin(-6000);
    chassis.moveToPoint(13, 17, 2000, {.maxSpeed = 80, .minSpeed = 40});

    // ? Ring 3
    chassis.swingToHeading(-135, lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(5, 500, {.maxSpeed = 60}, false);
    intake.spin();
    pros::delay(400);
    chassis.moveLinear(-3.0, 1000, {.maxSpeed = 30}, false);
    pros::delay(900);

    // ? Ring 4
    // intake.lift();
    // intake.spin(-6000);
    // chassis.moveLinear(6, 500, {.maxSpeed = 60}, false);
    // intake.spin();
    // pros::delay(800);
    // chassis.moveLinear(-15, 1000, {.maxSpeed = 30});
    // pros::delay(300);
    // intake.drop();

    // * Ladder
    chassis.turnToHeading(48, 1000, {.minSpeed = 20}, false);
    intake.stop();
    chassis.moveToPoint(55, 49, 2000, {.maxSpeed = 90}, false);
  }
  else
  {
    intake.enableSort(Intake::RingColors::RED);
    chassis.setPose({-48 - 7.00, 24 - 7.75, -90});
    chassis.turnToPoint(-72.0, 0, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(8, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-18, 2000, {.minSpeed = 30});
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
    // chassis.turnToPoint(24, 52, 1000, {.minSpeed = 20}, false);
    chassis.moveToPoint(-24, 52, 2000, {}, false);
    pros::delay(200);

    //* Corner
    chassis.turnToPoint(-13, 0, 1000, {.maxSpeed = 60, .minSpeed = 20}, false);
    pros::delay(200);
    arm.setIndex(0);
    intake.spin(-6000);
    chassis.moveToPoint(-13, 19, 2000, {.maxSpeed = 80, .minSpeed = 40});

    // ? Ring 3
    chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(5, 500, {.maxSpeed = 60}, false);
    intake.spin();
    pros::delay(400);
    chassis.moveLinear(-3.0, 1000, {.maxSpeed = 30}, false);
    pros::delay(400);

    // ? Ring 4
    intake.lift();
    intake.spin(-6000);
    chassis.moveLinear(6, 500, {.maxSpeed = 60}, false);
    intake.spin();
    pros::delay(800);
    chassis.moveLinear(-15, 1000, {.maxSpeed = 30});
    pros::delay(300);
    intake.drop();

    // * Ladder
    chassis.turnToHeading(-50, 1000, {.minSpeed = 20}, false);
    intake.stop();
    chassis.moveLinear(48, 2000, {});
  }
}

void debug(bool isRedTeam) { chassis.attemptReckonToGoal({72, 120, 0}, &clamp, 1000000); }

}  // namespace Autonomous