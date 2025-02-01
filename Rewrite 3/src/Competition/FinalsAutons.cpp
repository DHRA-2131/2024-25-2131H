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
    chassis.moveToPoint(-33, 35, 900, {.forwards = false}, true);
    doinkler.retract();

    // 1st goal
    chassis.turnToHeading(90, 1000, {}, true);
    doinkler.extend();
    chassis.turnToPoint(-48, 48, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();
    doinkler.retract();
    chassis.moveToPoint(-43, 43, 1000, {.forwards = false}, true);
    pros::delay(500);
    intake.spin();
    pros::delay(500);
    chassis.turnToPoint(-36, 0, 1000, {.forwards = false}, false);
    clamp.disableAutoClamp();

    // 2nd goal
    chassis.turnToPoint(-27, 55, 1000, {.forwards = false});
    chassis.moveToPoint(-35, 49.2, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();

    // 1st ring on 2nd goal
    chassis.turnToPoint(-12, 40, 1000);
    chassis.moveToPoint(-9, 45, 1000);

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
  else
  {
    // intake.enableSort(Intake::RingColors::BLUE);
    // Goal Rush
    chassis.setPose({-34.25, 144 - 17.5, 180});
    chassis.moveToPoint(-33, 144 - 52, 1000, {.minSpeed = 10}, true);
    doinkler.extend();
    chassis.moveToPoint(-33, 144 - 35, 900, {.forwards = false}, true);
    doinkler.retract();

    // 1st goal
    chassis.turnToHeading(90, 1000, {}, true);
    doinkler.extend();
    chassis.turnToPoint(-48, 144 - 48, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();
    doinkler.retract();
    chassis.moveToPoint(-42, 144 - 42, 1000, {.forwards = false}, true);
    pros::delay(500);
    intake.spin();
    pros::delay(500);
    chassis.turnToPoint(-36, 144 - 0, 1000, {.forwards = false}, false);
    clamp.disableAutoClamp();

    // 2nd goal
    chassis.turnToPoint(-27, 144 - 55, 1000, {.forwards = false});
    chassis.moveToPoint(-35, 144 - 49.2, 1000, {.forwards = false}, true);
    clamp.enableAutoClamp();

    // 1st ring on 2nd goal
    chassis.turnToPoint(-12, 144 - 40, 1000);
    chassis.moveToPoint(-9, 144 - 45, 1000);

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