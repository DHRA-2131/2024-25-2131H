#include <cmath>

#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace Autonomous
{

// Finals
void sixRingFinals(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    chassis.setPose({48 + 7.00, 24 - 7.75, 90});
    chassis.turnToPoint(72, 3.25, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(7.75, 1000, {});
    arm.setPosition(200);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30}, false);
    arm.setPosition(0);
    chassis.turnToPoint(48.5, 46, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(48.5, 46, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1 (Lader)
    chassis.turnToPoint(72 - 3.25 - 7.5 + 2, 72 - 3.25 - 7.25, 2000, {.minSpeed = 1});
    chassis.moveToPoint(72 - 3.25 - 7.5 + 2, 72 - 3.25 - 7.25, 2000, {}, false);
    doinklerLeft.extend();

    // ? Ring 2
    chassis.movePolar(-14, 45, 1000, {.minSpeed = 30}, false);
    chassis.turnToHeading(
        -75, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 20}, false);
    doinklerLeft.retract();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 10}, false);
    intake.spin();
    chassis.moveToPoint(32, 50, 2000, {.minSpeed = 10}, false);
    pros::Motor firstStage(-14);

    // * To Corner
    chassis.turnToHeading(5, 1000, {.minSpeed = 40});
    chassis.waitUntilDone();
    intake.spin();
    chassis.movePolar(-29, 5, 2000, {}, false);
    chassis.turnToHeading(-135, 2000, {.minSpeed = 20}, false);
    firstStage.move_voltage(-12000);
    chassis.movePolar(25.5, -138, 1000, {.maxSpeed = 50}, false, false);
    intake.spin();
    pros::delay(350);

    // ? Ring 4
    chassis.moveLinear(-4.5, 2000, {}, false);
    intake.lift();
    intake.spin();
    chassis.moveLinear(5, 1000, {.minSpeed = 20});
    firstStage.move_voltage(-12000);
    pros::delay(200);
    intake.spin();
    pros::delay(300);
    chassis.movePolar(-12, -135, 1000, {.minSpeed = 20}, false, false);
    intake.drop();

    chassis.turnToHeading(90, 2000, {.minSpeed = 1});

    // ? Ring 5
    chassis.movePolar(40, 87, 2000, {}, false);
    intake.lift();
    intake.spin();
  }
  else
  {
    chassis.setPose({-48 - 7.00 - 2, 24 - 7.75 + 1, -90});
    chassis.turnToPoint(-72, 3.25, 1000, {.minSpeed = 30}, false);
    chassis.moveLinear(7.6, 1000, {});
    arm.setPosition(210);

    // * Goal 1
    chassis.moveLinear(-17, 2000, {.minSpeed = 30}, false);
    arm.setPosition(0);
    chassis.turnToPoint(-48.5, 46, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(-48.5, 46, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);

    // ? Ring 1 (Lader)
    chassis.turnToPoint(-72 + 3.25 + 7.5 - 2, 72 - 3.25 - 7.25, 2000, {.minSpeed = 1});
    chassis.moveToPoint(-72 + 3.25 + 7.5 - 2, 72 - 3.25 - 7.25, 2000, {}, false);
    doinklerRight.extend();

    // ? Ring 2
    chassis.movePolar(-14, -45, 1000, {.minSpeed = 30}, false);
    chassis.turnToHeading(
        80,
        1000,
        {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 20},
        false);
    doinklerRight.retract();
    chassis.turnToHeading(95, 1000, {.minSpeed = 10}, false);
    intake.spin();
    chassis.moveToPoint(-32, 50, 2000, {.minSpeed = 10}, false);
    pros::Motor firstStage(-14);

    // * To Corner
    chassis.turnToHeading(0, 1000, {.minSpeed = 40});
    pros::delay(200);
    intake.stop();
    chassis.waitUntilDone();
    intake.spin();
    chassis.movePolar(-28, 0, 2000, {}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed = 20}, false);
    firstStage.move_voltage(-12000);
    chassis.movePolar(25.5, 138, 1000, {.maxSpeed = 50}, false, false);
    intake.spin();
    pros::delay(350);

    // ? Ring 4
    chassis.moveLinear(-4.5, 2000, {}, false);
    intake.lift();
    intake.spin();
    chassis.moveLinear(5, 1000, {.minSpeed = 20});
    firstStage.move_voltage(-12000);
    pros::delay(200);
    intake.spin();
    pros::delay(300);
    chassis.movePolar(-12, 135, 1000, {.minSpeed = 20}, false, false);
    intake.drop();

    chassis.turnToHeading(-90, 2000, {.minSpeed = 1});

    // ? Ring 5
    chassis.movePolar(40, -87, 2000, {}, false);
    intake.lift();
    intake.spin();
    pros::delay(200);
    intake.drop();
  }
}

void goalRush(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    // intake.enableSort(Intake::RingColors::BLUE);
    // Goal Rush
    chassis.setPose({-34.25, 17.5, 0});
    chassis.moveToPoint(-33, 52, 1000, {.minSpeed = 10}, true);
    doinklerRight.extend();
    chassis.moveToPoint(-32, 38 + 4.75, 900, {.forwards = false}, true);
    doinklerRight.retract();

    // 1st goal
    chassis.turnToPoint(-72, 72, 1000, {}, true);
    doinklerRight.extend();
    chassis.turnToPoint(-48, 48, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();
    doinklerRight.retract();
    chassis.moveToPoint(-50, 50, 1000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.moveLinear(5, 1000);
    pros::delay(100);
    intake.spin();
    pros::delay(500);
    chassis.turnToPoint(-36, 0, 1000, {.forwards = false}, false);
    clamp.disableAutoClamp();

    // 2nd goal
    chassis.turnToPoint(-27, 53 + 9, 1000, {.forwards = false});
    chassis.moveToPoint(-34, 46.8 + 8, 1000, {.forwards = false, .maxSpeed = 60}, true);
    clamp.enableAutoClamp();

    // 1st ring on 2nd goal
    chassis.turnToPoint(-12, 40 - 3, 1000);
    chassis.moveToPoint(-20, 45, 1000);

    // corner
    // chassis.turnToPoint(-24, 0, 1000, {.maxSpeed = 60, .minSpeed = 20}, false);
    // intake.spin(-6000);
    // chassis.moveToPoint(-11, 19 - 2, 2000, {.maxSpeed = 80, .minSpeed = 40});

    // // ? Ring 3
    // chassis.swingToHeading(130, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30}, false);
    // chassis.moveLinear(5 + 2.00, 500, {.maxSpeed = 50}, false);
    // intake.spin();
    // // ros::delay(4000);
    // chassis.moveLinear(-3.3 + 1, 1000, {.maxSpeed = 20}, false);
    // pros::delay(1300);
  }
  else
  {
    // intake.enableSort(Intake::RingColors::BLUE);
    // Goal Rush
    chassis.setPose({-34.25, 144 - 17.5, 180});
    chassis.moveToPoint(-33.2, 144 - 52, 1000, {.minSpeed = 10}, true);
    doinklerRight.extend();
    chassis.moveToPoint(-32, 144 - 38, 900, {.forwards = false}, true);
    doinklerRight.retract();

    // 1st goal
    chassis.turnToPoint(-72, 144 - 72, 1000, {}, true);
    doinklerRight.extend();
    chassis.turnToPoint(-48, 144 - 48, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();
    doinklerRight.retract();
    chassis.moveToPoint(-47, 144 - 47, 1000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.moveLinear(5, 1000);
    pros::delay(500);
    intake.spin();
    pros::delay(500);
    chassis.turnToPoint(-36, 144 - 0, 1000, {.forwards = false}, false);
    clamp.disableAutoClamp();

    // 2nd goal
    chassis.turnToPoint(-24, 84, 1000, {.forwards = false});
    chassis.moveToPoint(-23, 94, 1000, {.forwards = false, .maxSpeed = 60}, true);
    clamp.enableAutoClamp();

    // 1st ring on 2nd goal
    chassis.turnToPoint(-12, 108, 1000);
    intake.spin();
    chassis.moveToPoint(-13, 102, 1000, {}, true);
    pros::delay(1500);
    intake.stop();

    // corner
    // chassis.turnToPoint(-13, 0, 1000, {.maxSpeed = 60, .minSpeed = 20}, false);
    // intake.spin(-6000);
    // chassis.moveToPoint(-11, 19 - 2, 2000, {.maxSpeed = 80, .minSpeed = 40});

    // // ? Ring 3
    // chassis.swingToHeading(130, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30}, false);
    // chassis.moveLinear(5 + 2.00, 500, {.maxSpeed = 50}, false);
    // intake.spin();
    // // ros::delay(4000);
    // chassis.moveLinear(-3.3 + 1, 1000, {.maxSpeed = 20}, false);
    // pros::delay(1300);
  }
}

}  // namespace Autonomous