#include <cmath>

#include "2131H/Systems/Intake.hpp"
#include "2131H/Systems/Other.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

namespace Autonomous
{
void skills(bool isRedTeam)
{
  arm.enable();
  arm.setPosition(-10);
  chassis.setPose({72, 10.5, 0});

  // * Alliance Stake 1
  intake.spin();
  pros::delay(500);

  // * Goal 1
  chassis.moveToPoint(72, 22, 2000, {.minSpeed = 30});
  chassis.turnToPoint(51, 22, 2000, {.forwards = false, .minSpeed = 20});

  clamp.enableAutoClamp();
  chassis.moveToPoint(51, 24, 2000, {.forwards = false, .maxSpeed = 60});

  // ? Ring 1
  chassis.turnToPoint(48, 44, 2000, {.minSpeed = 30});
  chassis.moveToPoint(48, 44, 2000, {.minSpeed = 30});

  // ? Ring 2 (Wallstake)
  chassis.turnToPoint(12, 72, 2000, {.minSpeed = 30});
  chassis.moveLinear(36, 2000, {.minSpeed = 20});

  // * Wallstake
  arm.setIndex(1);
  chassis.turnToHeading(-90, 2000, {.minSpeed = 20}, false);
  intake.lift();
  chassis.moveLinear(6, 2000, {});
  pros::delay(800);
  intake.stop();
  arm.setIndex(2);

  // ? Ring 2 (Goal)
  pros::delay(1200);
  intake.drop();
  chassis.moveLinear(-12, 1000, {.minSpeed = 20}, false);
  arm.enable();

  chassis.turnToPoint(21, 96, 2000, {.minSpeed = 20}, false);
  intake.spin();
  chassis.moveToPoint(21, 96, 2000, {}, false);

  // ? Ring 3, 4, 5
  chassis.turnToHeading(-175, 2000, {.minSpeed = 10});
  chassis.movePolar(86, 180, 3000, {.maxSpeed = 100});

  // ? Ring 6
  chassis.turnToHeading(-45, 2000, {.minSpeed = 20});
  chassis.moveLinear(12, 1000, {}, false);

  // * Corner 1
  chassis.swingToHeading(45, lemlib::DriveSide::LEFT, 2000);
  clamp.disableAutoClamp();
}
}  // namespace Autonomous