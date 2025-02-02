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
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.setPose({-34.25, 17.5, 90});
  }
  else {}
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
    doinkler.extend();
    chassis.moveToPoint(-32, 38 + 4.75, 900, {.forwards = false}, true);
    doinkler.retract();

    // 1st goal
    chassis.turnToPoint(-72, 72, 1000, {}, true);
    doinkler.extend();
    chassis.turnToPoint(-48, 48, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();
    doinkler.retract();
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
    doinkler.extend();
    chassis.moveToPoint(-32, 144 - 38, 900, {.forwards = false}, true);
    doinkler.retract();

    // 1st goal
    chassis.turnToPoint(-72, 144 - 72, 1000, {}, true);
    doinkler.extend();
    chassis.turnToPoint(-48, 144 - 48, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();
    doinkler.retract();
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