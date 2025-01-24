#include <sys/types.h>

#include <cmath>

#include "2131H/Systems/Intake.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

namespace Autonomous
{

void goalRush(bool isRedTeam)
{
  arm.enable();
  if (isRedTeam)
  {
    // * Rush
    chassis.setPose({116 - 13.25 / 2, 19.5, 0});
    doinkler.extend();
    chassis.moveToPoint(chassis.getPose().x + 2, chassis.getPose().y + 33, 1020, {});
    pros::delay(810);
    doinkler.retract();
    chassis.waitUntilDone();

    // * Retreat
    chassis.moveToPoint(
        chassis.getPose().x, chassis.getPose().y - 13, 2000, {.forwards = false, .minSpeed = 20});
    pros::delay(300);
    doinkler.extend();
    chassis.waitUntilDone();
    pros::delay(400);
    doinkler.retract();

    // * Goal 1
    chassis.turnToPoint(96, 48, 2000, {.forwards = false, .minSpeed = 20});
    clamp.enableAutoClamp();
    chassis.moveToPoint(96, 48, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.swingToHeading(45, lemlib::DriveSide::LEFT, 1200, {.minSpeed = 50});

    // ? Ring 1
    intake.enableSort(Intake::RingColors::BLUE);
    intake.spin();
    chassis.waitUntilDone();
    clamp.disableAutoClamp();

    // * Goal 2
    chassis.turnToPoint(113, 56, 2000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    intake.stop();
    clamp.enableAutoClamp();
    chassis.moveToPoint(113, 56, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    intake.spin();

    // ? Ring 1
    chassis.swingToHeading(
        90,                       // * Changed
        lemlib::DriveSide::LEFT,  //
        2000,
        {lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});

    // * Corner
    chassis.moveToPoint(128, 24, 2000, {.forwards = false, .minSpeed = 40});
    chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPoint(144, 0, 2000, {.maxSpeed = 50});
    intake.spin(-4000);
    pros::delay(800);
    intake.spin();
    chassis.moveToPoint(
        chassis.getPose().x, chassis.getPose().y + 24, 1000, {.forwards = false, .maxSpeed = 70});
  }
  else
  {
    // * Rush
    chassis.setPose({-(116 + 13.25 + 2.5), 19.5, 0});
    doinkler.extend();
    chassis.moveToPoint(chassis.getPose().x + 2, chassis.getPose().y + 33, 1020, {});
    pros::delay(800);
    doinkler.retract();
    chassis.waitUntilDone();

    // * Retreat
    chassis.moveToPoint(
        chassis.getPose().x, chassis.getPose().y - 24, 2000, {.forwards = false, .minSpeed = 20});
    pros::delay(700);
    doinkler.extend();
    chassis.waitUntilDone();
    pros::delay(400);
    doinkler.retract();

    // * Goal 1
    chassis.turnToPoint(-124, 40, 2000, {.forwards = false, .minSpeed = 30});
    chassis.moveToPoint(-124, 40, 2000, {.forwards = false, .maxSpeed = 60});
    clamp.enableAutoClamp();
    chassis.moveToPoint(-96, 24, 2000);

    // ? Ring 1
    pros::delay(400);
    intake.spin();
    chassis.waitUntilDone();
    clamp.disableAutoClamp();

    // * Goal 2
    chassis.turnToPoint(chassis.getPose().x, 44, 2000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    intake.stop();
    clamp.enableAutoClamp();
    chassis.moveToPoint(chassis.getPose().x, 44, 2000, {.forwards = false});

    // ? Ring 1
    chassis.turnToPoint(-120, 50, 2000, {.minSpeed = 40});
    chassis.waitUntilDone();
    intake.spin();
    chassis.moveToPoint(-120, 50, 2000, {.minSpeed = 20});

    // * Corner
    chassis.turnToPoint(chassis.getPose().x, 14, 2000, {.minSpeed = 20});
    chassis.moveToPoint(chassis.getPose().x, 14, 2000);
  }
}

void soloWP(bool isRedTeam)
{
  if (isRedTeam)
  {  // * Alliance
    arm.setIndex(1);
    chassis.setPose({-53.31250, 24 - 13.25 / 2, -90});
    chassis.turnToPoint(-72 + 2, 1 + 1, 800);
    chassis.moveToPoint(-60 + 2, 12 + 1, 2000, {.minSpeed = 10});
    arm.setIndex(3);

    // * Goal 1
    chassis.moveToPoint(
        -36.5 + 4, 30 + 3, 800, {.forwards = false, .maxSpeed = 40, .minSpeed = 30}, false);
    chassis.moveToPoint(-36.5, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    arm.setIndex(0);
    chassis.turnToPoint(-48 + 4, 48 + 3, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.moveToPoint(-46 + 4, 46 + 3, 300, {.forwards = false, .maxSpeed = 100, .minSpeed = 30});
    chassis.moveToPoint(-46 + 4, 46 + 3, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Ring 1
    intake.spin();
    chassis.turnToPoint(-24 + 4, 46 + 3, 900, {.minSpeed = 40});
    chassis.moveToPoint(-24 + 4, 46 + 3, 2000, {.maxSpeed = 80, .minSpeed = 20}, false);
    pros::delay(300);

    // ! Store Ring 2
    chassis.turnToPoint(-60 + 12, 24, 1000, {.maxSpeed = 80, .minSpeed = 20}, false);
    chassis.moveToPoint(-60 + 12, 24, 2000);
    chassis.turnToPoint(-72 + 12, 24, 1000, {.minSpeed = 20}, false);
    clamp.disableAutoClamp(true);
    pros::delay(300);
    chassis.moveToPoint(-93 + 8, 24, 250, {.maxSpeed = 60, .minSpeed = 40});
    chassis.moveToPoint(-93 + 8, 24, 1700, {.maxSpeed = 40, .minSpeed = 10});

    int time = 0;
    pros::delay(900);
    intake.spin(2000);

    // * Goal 2
    chassis.turnToPoint(-99 + 14, 48, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    chassis.moveToPoint(-99 + 14, 48, 1000, {false, 80});

    // ? Ring 3
    chassis.moveToPoint(-123 + 14, 49.5 - 2, 3000, {.minSpeed = 20});
    pros::delay(300);
    intake.spin();
    chassis.waitUntilDone();
    pros::delay(400);

    // * TOUCH BAR
    chassis.turnToPoint(-86 + 8, 55 - 2, 3000, {.minSpeed = 20});
    chassis.moveToPoint(-86 + 8, 55 - 2, 3000, {.maxSpeed = 80});
  }
  else
  {
    // * Alliance
    arm.setIndex(1);
    chassis.setPose({53.3125, 24 - 13.25 / 2, 90});
    chassis.turnToPoint(72, 1, 800);
    chassis.moveToPoint(60, 12, 2000, {.minSpeed = 10});
    arm.setIndex(3);

    // * Goal 1
    chassis.moveToPoint(36.5, 30, 800, {.forwards = false, .maxSpeed = 20, .minSpeed = 10}, false);
    chassis.moveToPoint(36.5, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    arm.setIndex(0);
    chassis.turnToPoint(48, 48, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    intake.enableSort(Intake::RingColors::RED);
    chassis.moveToPoint(46, 46, 300, {.forwards = false, .maxSpeed = 100, .minSpeed = 30});
    chassis.moveToPoint(46, 46, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Ring 1
    intake.spin();
    chassis.turnToPoint(24, 46, 900, {.minSpeed = 40});
    chassis.moveToPoint(24, 46, 2000, {.maxSpeed = 80, .minSpeed = 20}, false);
    pros::delay(300);

    // ! Store Ring 2
    chassis.turnToPoint(60, 24, 1000, {.maxSpeed = 80}, false);
    chassis.moveToPoint(60, 24, 2000);
    chassis.turnToPoint(72, 24, 1000, {.minSpeed = 20}, false);
    clamp.disableAutoClamp(true);
    pros::delay(300);
    chassis.moveToPoint(93, 24, 1700, {.maxSpeed = 40, .minSpeed = 10});

    int time = 0;
    while ((intake.getCurrentRingColor() != Intake::RingColors::BLUE) && time < 980)
    {
      time += 10;
      pros::delay(10);
    }
    intake.spin(2000);

    // * Goal 2
    chassis.turnToPoint(99, 48, 1000, {false});
    clamp.enableAutoClamp();
    chassis.moveToPoint(99, 48, 1000, {false, 80});

    // ? Ring 3
    chassis.moveToPoint(123, 49.5, 3000, {.minSpeed = 20});
    pros::delay(300);
    intake.spin();
    chassis.waitUntilDone();
    pros::delay(400);

    // * TOUCH BAR
    chassis.turnToPoint(86, 55, 3000, {.minSpeed = 20});
    chassis.moveToPoint(86, 55, 3000, {.maxSpeed = 80});
  }
  chassis.waitUntilDone();
}

void ringRush(bool isRedTeam)
{
  if (isRedTeam)
  {
    chassis.setPose({-97.8, 21.67, -22});  // Set the robot's position
    rush.extend();                         // Extend the rush mechanism

    chassis.moveToPoint(
        (chassis.getPose().x + std::sin(chassis.getPose(true).theta) * 41.5),
        (chassis.getPose().y + std::cos(chassis.getPose(true).theta) * 41.5),
        2000,
        {},
        false);  // Move to the ring stack

    chassis.moveToPoint(
        (chassis.getPose().x + std::sin(chassis.getPose(true).theta) * -21),
        (chassis.getPose().y + std::cos(chassis.getPose(true).theta) * -21),
        2000,
        {false},
        false);  // Move to the ring stack

    rush.retract();  // Retract the rush mechanism

    pros::delay(200);  // Wait for rush to retract

    chassis.moveToPoint(
        (chassis.getPose().x + std::sin(chassis.getPose(true).theta) * -5),
        (chassis.getPose().y + std::cos(chassis.getPose(true).theta) * -5),
        2000,
        {false},
        false);  // Move to the ring stack

    // * Goal 1
    clamp.enableAutoClamp();
    chassis.turnToPoint(-91, 50, 1000, {false});
    chassis.moveToPoint(-91, 50, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Rings 1, 2, 3
    chassis.turnToPoint(-125, 46, 2200, {.minSpeed = 40});  // ! Changed
    chassis.moveToPoint(-125, 46, 2200, {.maxSpeed = 40});
    intake.enableSort(Intake::RingColors::BLUE);
    intake.spin();
    chassis.waitUntilDone();
    pros::delay(600);
    chassis.moveToPoint(-119, 46, 1000, {.forwards = false});  // Back up

    // * Alliance Stake
    chassis.turnToPoint(-84, 20, 1000, {.maxSpeed = 80, .minSpeed = 10});
    chassis.moveToPoint(-84, 20, 3000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(600);
    // arm.setIndex(1);
    intake.stop();
    chassis.moveToPoint(-48, 13.9, 3000, {.minSpeed = 20});  // ! Changed
    // chassis.turnToHeading(-180, 1000, {});                   // ! Changed
    // pros::delay(800);
    // intake.stop();
    // chassis.waitUntilDone();

    // arm.setPosition(205);
    // pros::delay(1000);

    // * Touch Bar
    // chassis.moveToPoint(-96, 48, 2000, {.forwards = false, .minSpeed = 40});
    // pros::delay(200);
    // arm.setPosition(0);
    // chassis.turnToPoint(-88, 52, 1000, {.minSpeed = 20});
    // chassis.moveToPoint(-88, 52, 2000);
  }
  else
  {
    chassis.setPose({97.8, 21.67, 22});  // Set the robot's position
    rush.extend();                       // Extend the rush mechanism
    chassis.moveToPoint(
        chassis.getPose().x + std::sin(chassis.getPose(true).theta) * 42,
        chassis.getPose().y + std::cos(chassis.getPose(true).theta) * 42,
        2000,
        {},
        false);  // Move to the ring stack

    chassis.moveToPoint(
        chassis.getPose().x + std::sin(chassis.getPose(true).theta) * -21,
        chassis.getPose().y + std::cos(chassis.getPose(true).theta) * -21,
        2000,
        {false},
        false);  // Move to the ring stack

    rush.retract();  // Retract the rush mechanism

    pros::delay(200);  // Wait for rush to retract

    chassis.moveToPoint(
        chassis.getPose().x + std::sin(chassis.getPose(true).theta) * -5,
        chassis.getPose().y + std::cos(chassis.getPose(true).theta) * -5,
        2000,
        {false},
        false);  // Move to the ring stack

    // * Goal 1
    clamp.enableAutoClamp();
    chassis.turnToPoint(91, 54, 1000, {false});
    chassis.moveToPoint(91, 54, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Rings 1, 2, 3
    chassis.moveToPoint(130, 52, 2200, {.maxSpeed = 40});
    intake.enableSort(Intake::RingColors::RED);
    intake.spin();
    chassis.waitUntilDone();
    pros::delay(600);

    // * Alliance Stake
    chassis.moveToPoint(119, 52, 1000, {.forwards = false});
    chassis.turnToPoint(87, 24, 1000, {.maxSpeed = 80, .minSpeed = 10});
    chassis.moveToPoint(87, 24, 3000, {.maxSpeed = 90});
    arm.setIndex(1);
    pros::delay(1500);
    intake.stop();

    chassis.moveToPoint(83.3, 16.5, 3000, {.minSpeed = 10});
    chassis.turnToPoint(75.3, 1, 1000, {.minSpeed = 5}, false);

    arm.setPosition(200);
    pros::delay(800);

    // * Touch Bar
    chassis.moveToPoint(96, 48, 2000, {.forwards = false, .minSpeed = 40});
    pros::delay(200);
    arm.setPosition(0);
    chassis.turnToPoint(90, 56, 1000, {.minSpeed = 20});
    chassis.moveToPoint(90, 56, 2000);
  }
  chassis.waitUntilDone();
}

void safeRingSide(bool isRedTeam)
{
  if (isRedTeam)
  {
    // * Alliance
    arm.enable();
    arm.setIndex(1);
    chassis.setPose({48 + 7.25, 24 - 7.75, 90});
    chassis.turnToPoint(72 + 2, 1 - 1, 800);
    chassis.moveToPoint(60, 12 - 1, 2000, {.minSpeed = 10});
    arm.setIndex(3);

    // * Goal 1
    chassis.moveToPoint(
        48, 30 + 3, 800, {.forwards = false, .maxSpeed = 40, .minSpeed = 30}, false);
    chassis.moveToPoint(48, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    arm.setIndex(0);
    chassis.turnToPoint(52, 51, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    // intake.enableSort(isRedTeam ? Intake::RingColors::BLUE : Intake::RingColors::RED);
    chassis.moveToPoint(50, 49, 300, {.forwards = false, .maxSpeed = 100, .minSpeed = 30});
    chassis.moveToPoint(50, 49, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Ring 1
    chassis.moveToPoint(59, 33, 1200, {.minSpeed = 20});
    chassis.turnToHeading(89, 1200, {.minSpeed = 20}, false);
    doinkler.extend();
    pros::delay(200);

    // ? Ring 2
    chassis.moveToPoint(49, 44, 2000, {.maxSpeed = 80, .minSpeed = 30});
    chassis.turnToHeading(-120, 1200, {.minSpeed = 20}, false);
    doinkler.retract();
    pros::delay(300);
    intake.spin();
    chassis.turnToPoint(24, 44, 900, {.minSpeed = 40});
    chassis.moveToPoint(24, 44, 2000, {.maxSpeed = 75, .minSpeed = 20});
    chassis.swingToHeading(-160, lemlib::DriveSide::LEFT, 1200, {.minSpeed = 40});

    // // ? Ring 3 (Corner)
    // chassis.moveToPoint(12, 13, 500, {.minSpeed = 60});
    // chassis.moveToPoint(12, 13, 2000, {.maxSpeed = 30, .minSpeed = 20});
    // chassis.swingToHeading(-125, lemlib::DriveSide::RIGHT, 1300);

    // * TOUCH BAR
    // chassis.moveToPoint(15, 15, 2500, {.forwards = false, .maxSpeed = 5});
    // chassis.moveToPoint(36, 48, 2000, {.forwards = false});
    chassis.turnToPoint(48 + 13.435 + 1, 42 + 13.435, 2000, {.forwards = false, .minSpeed = 20});
    chassis.moveToPoint(48 + 13.435 + 1, 42 + 13.435, 2000, {.forwards = false});
  }
  else
  {  // * Alliance
    arm.enable();
    arm.setIndex(1);
    chassis.setPose({-48 - 7.25, 24 - 7.75, -90});
    chassis.turnToPoint(-72, 0, 800);
    chassis.moveToPoint(-60, 12 - 1, 2000, {.minSpeed = 10});
    arm.setIndex(3);
    chassis.waitUntilDone();

    // * Goal 1
    chassis.moveToPoint(
        -46, 30 + 3, 800, {.forwards = false, .maxSpeed = 40, .minSpeed = 30}, false);
    chassis.moveToPoint(-46, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    arm.setIndex(0);
    chassis.turnToPoint(-46, 42, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    // intake.enableSort(isRedTeam ? Intake::RingColors::BLUE : Intake::RingColors::RED);
    chassis.moveToPoint(-46, 42, 300, {.forwards = false, .maxSpeed = 100, .minSpeed = 30});
    chassis.moveToPoint(-46, 42, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Ring 1
    chassis.moveToPoint(-56, 20, 1200, {.minSpeed = 20});
    chassis.turnToHeading(-94, 1200, {.minSpeed = 20}, false);
    doinkler.extend();
    pros::delay(200);

    // ? Ring 2
    chassis.moveToPoint(-49, 44, 2000, {.maxSpeed = 80, .minSpeed = 30});
    chassis.turnToHeading(60, 1200, {.minSpeed = 20}, false);
    doinkler.retract();
    pros::delay(300);
    intake.spin();
    chassis.turnToPoint(-24, 46, 900, {.minSpeed = 40});
    chassis.moveToPoint(-24, 46, 2000, {.maxSpeed = 80, .minSpeed = 20});
    chassis.swingToHeading(160, lemlib::DriveSide::RIGHT, 1200, {.minSpeed = 40});

    // // ? Ring 3 (Corner)
    // chassis.moveToPoint(-9, 13, 500, {.minSpeed = 60});
    // chassis.moveToPoint(-9, 13, 1000, {.maxSpeed = 30, .minSpeed = 20});
    // chassis.swingToHeading(125, lemlib::DriveSide::LEFT, 500, {}, false);

    // * TOUCH BAR
    // pros::delay(500);
    // chassis.moveToPoint(-36, 48, 2500, {.forwards = false, .maxSpeed = 15});
    // chassis.moveToPoint(-36, 48, 2000, {.forwards = false});
    chassis.turnToPoint(-48 - 13.435 - 1, 46 + 13.435, 2000, {.forwards = false, .minSpeed = 20});
    chassis.moveToPoint(-48 - 13.435 - 1, 46 + 13.435, 2000, {.forwards = false});
  }
}

void safeGoalSide(bool isRedTeam)
{
  if (isRedTeam) { safeRingSide(false); }
  else { safeRingSide(true); }
}

void debug(bool isRedTeam)
{
  chassis.setPose({0, 0, 0});
}

}  // namespace Autonomous