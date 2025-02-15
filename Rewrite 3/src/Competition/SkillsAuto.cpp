#include <cmath>

#include "2131H/Systems/Intake.hpp"
#include "2131H/Systems/Other.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
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
  chassis.turnToPoint(28, 70, 2000, {.minSpeed = 10});
  chassis.moveToPoint(14, 70, 2000, {});

  // * Wallstake 1
  pros::delay(300);
  arm.setIndex(1);
  chassis.turnToHeading(-92, 2000, {.minSpeed = 20}, false);
  intake.lift();
  chassis.movePolar(8, -90, 1500, {});

  pros::delay(400);
  intake.spin(100);
  arm.setIndex(2);
  chassis.waitUntilDone();
  pros::delay(300);
  // chassis.shimmy(300, 150);

  // ? Ring 2 (Goal)
  intake.drop();
  chassis.moveLinear(-11, 1000, {.minSpeed = 1}, false);
  arm.enable();

  chassis.turnToHeading(0, 2000, {.minSpeed = 1}, false);
  intake.spin();
  chassis.movePolar(24, 0, 2000, {}, false);

  // ? Ring 3, 4, 5
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80, .minSpeed = 1});
  chassis.movePolar(48, -180, 2000, {.maxSpeed = 100, .minSpeed = 60});
  chassis.movePolar(36, -180, 3000, {.maxSpeed = 60});

  // ? Ring 6
  chassis.turnToHeading(-60, 1000, {.minSpeed = 20});
  chassis.movePolar(14, -60, 1000, {}, false);

  // * Corner 1
  chassis.turnToHeading(30, 2000, {.minSpeed = 20}, false);
  chassis.moveLinear(-20, 1000, {}, false);
  intake.stop();
  clamp.disableAutoClamp();

  // ! Tare Corner 1
  chassis.setPose({19.5, 17.25, chassis.getPose().theta});

  // * Goal 2
  chassis.moveLinear(8, 2000);
  chassis.moveToPoint(72, 24, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 50}, false);
  clamp.enableAutoClamp();
  chassis.moveToPoint(94, 24, 2000, {.forwards = false, .maxSpeed = 30}, false);

  // ? Ring 1
  chassis.turnToPoint(144 - 48 - 2, 44, 2000, {.minSpeed = 30}, false);
  intake.spin();
  chassis.moveToPoint(144 - 48 - 2, 44, 2000, {.minSpeed = 30});

  // ? Ring 2 (Wallstake)
  chassis.turnToPoint(144 - 28, 71, 2000, {.minSpeed = 10});
  chassis.moveToPoint(144 - 14, 71, 2000, {});

  // * Wallstake 2
  pros::delay(300);
  arm.setIndex(1);
  chassis.turnToHeading(92, 2000, {.minSpeed = 20}, false);
  intake.lift();
  chassis.movePolar(8, 90, 1500, {});
  intake.stop();
  arm.setIndex(2);
  chassis.waitUntilDone();
  pros::delay(300);
  // chassis.shimmy(300, 150);

  // ? Ring 2 (Goal)
  intake.drop();
  chassis.moveLinear(-11, 1000, {.minSpeed = 1}, false);
  arm.disable();

  chassis.turnToHeading(0, 2000, {.minSpeed = 1}, false);
  intake.spin();
  chassis.movePolar(24, 0, 2000, {}, false);

  // ? Ring 3, 4, 5
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80, .minSpeed = 1});
  chassis.movePolar(48, -180, 2000, {.maxSpeed = 100, .minSpeed = 60});
  chassis.movePolar(36, -180, 3000, {.maxSpeed = 60});

  // ? Ring 6
  chassis.turnToHeading(60, 800, {.minSpeed = 20});
  chassis.movePolar(14, 60, 1000, {}, false);

  // * Corner 2
  chassis.turnToHeading(-30, 2000, {.minSpeed = 20}, false);
  chassis.moveLinear(-20, 1200, {}, false);
  intake.stop();
  clamp.disableAutoClamp();

  // ! Tare Corner 2
  chassis.setPose({132.5, 15, chassis.getPose().theta});

  // * Goal 3
  chassis.movePolar(5, -33, 2000, {}, false);
  chassis.moveToPoint(120, 66, 2000, {.maxSpeed = 100, .minSpeed = 30}, false);
  pros::Motor firstStage(-14, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
  firstStage.move_voltage(12000);

  chassis.moveToPoint(96, 96, 3000, {.maxSpeed = 60}, false);

  chassis.turnToPoint(75, 118, 1000, {.forwards = false, .minSpeed = 1}, false);
  clamp.enableAutoClamp();
  chassis.moveToPoint(75, 118, 3000, {.forwards = false, .maxSpeed = 50}, false);
  intake.spin();
  chassis.setPose({70, 124, chassis.getPose().theta});
  arm.setIndex(4);

  // ? Ring 1
  chassis.turnToPoint(49, 97, 1000, {.minSpeed = 20});
  chassis.moveToPoint(49, 97, 2000, {.minSpeed = 10});

  // ? Ring 2
  chassis.turnToPoint(76, 74, 2000, {.minSpeed = 1});
  chassis.moveToPoint(76, 74, 2000, {}, false);
  chassis.turnToHeading(45, 1000, {.minSpeed = 10});
  chassis.movePolar(24, 45, 1000, {.minSpeed = 10});
  chassis.turnToPoint(132.5, 104, 2000, {.minSpeed = 1}, false);
  pros::delay(400);  // Wait for ring to score

  // ? Ring 3
  chassis.moveToPoint(132.5, 104, 2000, {.minSpeed = 1});
  chassis.swingToHeading(0, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30});
  chassis.moveToPoint(129, 134, 2000, {.maxSpeed = 50});
  pros::delay(1100);
  intake.stop();

  // * Corner 3
  chassis.turnToHeading(-100, 1000, {.maxSpeed = 80, .minSpeed = 20}, false);
  clamp.disableAutoClamp();

  // * Corner 4
  chassis.moveToPoint(96, 115, 2000, {.minSpeed = 1});
  pros::delay(200);
  intake.spin();
  chassis.turnToHeading(-90, 2000, {.minSpeed = 1});
  chassis.movePolar(40, -90, 2000);

  // * Goal 3
  chassis.turnToHeading(180, 1000, {}, false);
  clamp.enableAutoClamp();
  chassis.moveLinear(-15, 1000);
  chassis.turnToHeading(135, 2000, {.minSpeed = 50});
  chassis.movePolar(-30, 102, 2000);
  pros::delay(500);
  clamp.disableAutoClamp();

  // * Hang
  chassis.turnToHeading(145, 2000, {.minSpeed = 20});
  chassis.movePolar(44, 145, 2000, {.minSpeed = 1}, false);
  chassis.turnToHeading(-45, 2000);
  arm.setIndex(4);

  chassis.movePolar(-20, -45, 2000, {.maxSpeed = 80}, false);
}
}  // namespace Autonomous