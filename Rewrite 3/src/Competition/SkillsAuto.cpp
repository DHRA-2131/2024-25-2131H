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
  intake.spin(-6000);

  // * Goal 1
  chassis.moveToPoint(72, 22, 2000, {.minSpeed = 30});
  chassis.turnToPoint(51, 22, 2000, {.forwards = false, .minSpeed = 20}, false);
  clamp.enableAutoClamp();
  chassis.moveToPoint(51, 24, 2000, {.forwards = false, .maxSpeed = 60}, false);
  intake.spin();

  // ? Ring 1
  chassis.turnToPoint(48, 44, 2000, {.minSpeed = 30});
  chassis.moveToPoint(48, 44, 2000, {.minSpeed = 30});

  // ? Ring 2 (Wallstake)
  chassis.turnToPoint(28, 71, 2000, {.minSpeed = 10});
  chassis.moveToPoint(14, 71, 2000, {});

  // * Wallstake
  pros::delay(300);
  arm.setIndex(1);
  chassis.turnToHeading(-93, 2000, {.minSpeed = 20}, false);
  intake.lift();
  chassis.moveLinear(8, 1500, {});
  pros::delay(400);
  intake.stop();
  arm.setIndex(2);
  pros::delay(500);

  // ? Ring 2 (Goal)
  intake.drop();
  chassis.moveLinear(-12, 1000, {.minSpeed = 1}, false);
  arm.enable();

  chassis.turnToHeading(0, 2000, {.minSpeed = 1}, false);
  intake.spin();
  chassis.movePolar(24, 0, 2000, {}, false);

  // ? Ring 3, 4, 5
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80, .minSpeed = 1});
  chassis.movePolar(48, -180, 2000, {.maxSpeed = 100, .minSpeed = 60});
  chassis.movePolar(36, -180, 3000, {.maxSpeed = 60});

  // ? Ring 6
  chassis.turnToHeading(-45, 800, {.minSpeed = 20});
  chassis.movePolar(14, -45, 1000, {}, false);

  // * Corner 1
  chassis.turnToHeading(30, 2000, {.minSpeed = 20}, false);
  chassis.moveLinear(-11, 1000, {}, false);
  intake.stop();
  clamp.disableAutoClamp();

  // ! Tare Corner 1
  chassis.setPose({18.5, 18, chassis.getPose().theta});

  // * Goal 2
  chassis.moveLinear(8, 2000);
  chassis.moveToPoint(72, 24, 2000, {.forwards = false, .minSpeed = 50}, false);
  clamp.enableAutoClamp();
  chassis.moveToPoint(94, 24, 2000, {.forwards = false, .maxSpeed = 30}, false);

  // ? Ring 1
  chassis.turnToPoint(144 - 48, 44, 2000, {.minSpeed = 30}, false);
  intake.spin();
  chassis.moveToPoint(144 - 48, 44, 2000, {.minSpeed = 30});

  // ? Ring 2 (Wallstake)
  chassis.turnToPoint(144 - 28, 71, 2000, {.minSpeed = 10});
  chassis.moveToPoint(144 - 14, 71, 2000, {});

  // * Wallstake
  pros::delay(300);
  arm.setIndex(1);
  chassis.turnToHeading(93, 2000, {.minSpeed = 20}, false);
  intake.lift();
  chassis.moveLinear(8, 1500, {});
  pros::delay(400);
  intake.stop();
  arm.setIndex(2);
  pros::delay(500);

  // ? Ring 2 (Goal)
  intake.drop();
  chassis.moveLinear(-12, 1000, {.minSpeed = 1}, false);
  arm.enable();

  chassis.turnToHeading(0, 2000, {.minSpeed = 1}, false);
  intake.spin();
  chassis.movePolar(24, 0, 2000, {}, false);

  // ? Ring 3, 4, 5
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80, .minSpeed = 1});
  chassis.movePolar(48, -180, 2000, {.maxSpeed = 100, .minSpeed = 60});
  chassis.movePolar(36, -180, 3000, {.maxSpeed = 60});

  // ? Ring 6
  chassis.turnToHeading(50, 800, {.minSpeed = 20});
  chassis.movePolar(14, 50, 1000, {}, false);

  // * Corner 1
  chassis.turnToHeading(-30, 2000, {.minSpeed = 20}, false);
  chassis.moveLinear(-11, 1000, {}, false);
  intake.stop();
  clamp.disableAutoClamp();
}
}  // namespace Autonomous