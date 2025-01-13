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
  arm.setPosition(-10);  
  // * Alliance stake
  intake.spin();
  chassis.setPose({72, 14.25, 0});
  pros::delay(400);

  // * Goal 1
  clamp.enableAutoClamp();
  chassis.moveToPoint(72, 24, 1000, {.minSpeed = 10}, false);
  intake.stop();
  chassis.turnToPoint(96, 24, 1000, {.forwards = false, .minSpeed = 30});
  chassis.moveToPoint(96, 24, 1000, {.forwards = false, .maxSpeed = 70});

  // ? Ring 1
  chassis.turnToPoint(92, 48, 1000, {.maxSpeed = 63, .minSpeed = 30});
  intake.spin();
  chassis.moveToPoint(92, 48, 1300, {.minSpeed = 20});

  // ? Ring 2 (By wallstake)
  chassis.turnToPoint(124.75, 72.5, 1000, {.minSpeed = 30});
  chassis.moveToPoint(124.75, 72.5, 1000, {}, false);
  pros::delay(500);

  // ? Ring 3
  chassis.swingToHeading(-180, lemlib::DriveSide::LEFT, 1000, {});
  chassis.moveToPoint(120, 48, 1000, {.minSpeed = 60});

  // ? Ring 4 & 5
  chassis.moveToPoint(120, 12, 1000, {.maxSpeed = 60});

  // ? Ring 6 & CORNER (Shimmy shimmy)
  chassis.swingToHeading(135, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 60});
  chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 1000);
  chassis.swingToHeading(-45, lemlib::DriveSide::RIGHT, 1000, {}, false);
  intake.spin(-6000);
  pros::delay(100);
  clamp.disableAutoClamp();

  // ! Tare Position
  chassis.setPose({135.5, 14.75, chassis.getPose().theta});

  // * Goal 2
  chassis.moveToPoint(74, 24, 900, {.minSpeed = 30}, false);
  chassis.moveToPoint(74, 24, 1000, {.maxSpeed = 50, .minSpeed = 30}, false);
  chassis.turnToPoint(48, 24, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20});
  clamp.enableAutoClamp();
  chassis.moveToPoint(48, 24, 2000, {.forwards = false, .maxSpeed = 70});

  // ? Ring 1
  chassis.turnToPoint(144 - 92 - 5, 48, 2000, {.maxSpeed = 63, .minSpeed = 30});
  intake.spin();
  chassis.moveToPoint(144 - 92 - 5, 48, 1300, {.minSpeed = 20});

  // ? Ring 2 (By wallstake)
  chassis.turnToPoint(144 - 125 - 5, 72.5 - 2, 1000, {.minSpeed = 30});
  chassis.moveToPoint(144 - 125 - 5, 72.5 - 2, 1000, {}, false);
  pros::delay(500);

  // ? Ring 3
  chassis.swingToHeading(175, lemlib::DriveSide::RIGHT, 1000, {});
  chassis.moveToPoint(26, 48, 1000, {.minSpeed = 60});

  // ? Ring 4 & 5
  chassis.moveToPoint(26, 13.5, 2000, {.maxSpeed = 60});

  // ? Ring 6 & CORNER (Shimmy shimmy)
  chassis.swingToHeading(-135, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 60});
  chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 1000);
  chassis.swingToHeading(45, lemlib::DriveSide::LEFT, 1000, {}, false);
  intake.spin(-6000);
  pros::delay(100);
  clamp.disableAutoClamp();

  // ! Tare Position
  chassis.setPose({11.5, 14.5, chassis.getPose().theta});

  // ? Ring 1 (Store)
  chassis.moveToPoint(24, 54, 2000, {.minSpeed = 40});
  intake.spin(6000);
  intake.enableSort(Intake::RingColors::BLUE);
  chassis.moveToPoint(48, 96, 2000, {.minSpeed = 20});
  int time = 0;
  while ((intake.getCurrentRingColor() != Intake::RingColors::RED) && time < 1200)
  {
    time += 10;
    pros::delay(10);
  }
  pros::delay(300);
  intake.stop();

  // * Goal 3
  chassis.turnToPoint(72, 118, 1200, {.forwards = false, .maxSpeed = 80, .minSpeed = 30});
  clamp.enableAutoClamp();
  chassis.moveToPoint(72, 118, 1200, {.forwards = false, .maxSpeed = 70}, false);

  // ? Ring 2
  intake.spin();
  chassis.turnToPoint(92, 96, 1200, {.minSpeed = 30});
  chassis.moveToPoint(92, 96, 2000, {.minSpeed = 20});

  // ? Ring 3
  chassis.turnToPoint(120, 94, 1200, {.minSpeed = 30});
  chassis.moveToPoint(120, 96, 2000, {.maxSpeed = 90, .minSpeed = 20}, false);

  // // * Wall Stake
  // arm.setIndex(1);
  // chassis.waitUntilDone();
  // pros::delay(800);
  // intake.stop();
  // chassis.turnToPoint(120, 72, 1200, {.minSpeed = 20});
  // chassis.moveToPoint(120, 73, 2000, {.minSpeed = 20});
  // chassis.turnToPoint(144, 73, 1000, {.minSpeed = 20});
  // chassis.moveToPoint(138, 72, 3000, {}, false);
  // arm.setIndex(3);
  // chassis.waitUntilDone();

  // ? Ring 4
  chassis.turnToPoint(130, 120 + 2, 1200, {.minSpeed = 30});
  chassis.moveToPoint(130, 120 + 2, 2000, {.maxSpeed = 70});

  // * CORNER
  chassis.turnToHeading(5, 1200, {.minSpeed = 20}, false);
  // doinkler.extend();
  pros::delay(400);
  intake.stop();
  chassis.swingToHeading(-100, lemlib::DriveSide::LEFT, 1200, {.minSpeed = 20});
  chassis.swingToHeading(-110, lemlib::DriveSide::RIGHT, 800, {.minSpeed = 30});
  pros::delay(400);
  clamp.disableAutoClamp();

  // * Goal 4
  chassis.swingToHeading(224, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 40}, false);
  intake.spin();
  doinkler.retract();
  chassis.moveToPoint(96, 108 + 2, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 20});
  chassis.moveToPoint(72, 120, 2000, {.minSpeed = 40});
  chassis.moveToPoint(48, 112 + 2, 2000, {.minSpeed = 10});
  chassis.turnToPoint(48, 124 + 2, 1000, {.forwards = false});
  clamp.enableAutoClamp();
  chassis.moveToPoint(48, 124, 1000, {.forwards = false});
  time = 0;
  while (!clamp.isGoal() && time < 1200)
  {
    pros::delay(10);
    time += 10;
  }
  chassis.setPose({48, 118, chassis.getPose().theta});
  arm.enable();
  arm.setIndex(1);

  // * CORNER
  chassis.moveToPoint(17, 137, 2000, {.forwards = false, .minSpeed = 20});
  arm.setIndex(2);
  clamp.disableAutoClamp();

  // * Hang
  chassis.swingToHeading(135, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 40});
  arm.setIndex(3);
  chassis.moveToPoint(48, 96, 1200);
  chassis.turnToHeading(-45, 1200, {}, false);
  intake.stop();
  chassis.moveToPoint(
      chassis.getPose().x + std::cos(chassis.getPose(true).theta) * 40,
      chassis.getPose().y + std::sin(chassis.getPose(true).theta) * 40,
      3000,
      {false, 30});

  chassis.waitUntilDone();
}
}  // namespace Autonomous