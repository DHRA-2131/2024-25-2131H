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
  if (isRedTeam)
  {
    arm.setIndex(1);                         // Set the arm to the load position
    chassis.setPose({144 - 11.5, 19.5, 0});  // Set the robot's position

    chassis.moveToPoint(
        144 - 11.5,
        50,
        2000,              //
        {true, 100, 40});  // Go next to the ring stack

    chassis.swingToPoint(
        144 - 25.5,
        72,
        lemlib::DriveSide::LEFT,
        2000,               //
        {.minSpeed = 30});  // Point towards the goal

    chassis.moveToPoint(144 - 17, 57.5, 1000);  // Move to the goal
    arm.setPosition(200);                       // Score preload

    pros::delay(300);  // Make sure Arm gets there first
    rush.extend();     // Extend the rush mechanism
    pros::delay(400);  // Wait for rush to extend

    chassis.moveToPoint(144 - 13, 25, 2000, {.forwards = false});  // Retreat
    pros::delay(400);
    arm.setPosition(250);  // Fully score the arm
    pros::delay(800);
    arm.setPosition(55);  // Reset Arm
    rush.retract();       // Reset Rush
    pros::delay(400);

    // Goal 2
    clamp.enableAutoClamp();
    chassis.turnToPoint(144 - 30, 24, 800, {.minSpeed = 30});
    chassis.moveToPoint(144 - 30, 24, 1000, {.minSpeed = 10});
    chassis.turnToPoint(144 - 46, 43, 1000, {.forwards = false});
    chassis.moveToPoint(144 - 46, 43, 1000, {.forwards = false, .maxSpeed = 80});
    intake.enableSort(Intake::RingColors::BLUE);  // Sort blue rings
    pros::delay(100);
    intake.spin();
    chassis.turnToPoint(144 - 24, 43, 1000, {.minSpeed = 20});  // Grab Ring
    chassis.moveToPoint(144 - 24, 43, 1000, {.maxSpeed = 110, .minSpeed = 20});

    chassis.moveToPoint(144 - 48, 48, 1000, {false, 60});
    chassis.turnToPoint(144 - 0, 0, 800);
    chassis.moveToPoint(144 - 9.369, 9.369 - 1.2, 800, {.minSpeed = 40}, false);
    chassis.moveToPoint(144 - 9.369, 9.369 - 1.2, 1000, {.maxSpeed = 40}, false);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 700, {.minSpeed = 80});
    pros::delay(700);

    chassis.setPose(144 - 10, 12, chassis.getPose().theta);
    chassis.moveToPoint(144 - 24, 24, 800, {.forwards = false, .maxSpeed = 40}, false);
    arm.setPosition(0);  // Reset Arm
    chassis.turnToPoint(144 - 57, 59, 2000, {.maxSpeed = 50, .minSpeed = 30});
    chassis.moveToPoint(144 - 57, 59, 2000, {}, false);
    chassis.waitUntilDone();
  }
  else
  {
    arm.setIndex(1);                   // Set the arm to the load position
    chassis.setPose({11.5, 19.5, 0});  // Set the robot's position

    chassis.moveToPoint(
        11.5,
        50,
        2000,              //
        {true, 100, 40});  // Go next to the ring stack

    chassis.swingToPoint(
        24,
        72,
        lemlib::DriveSide::RIGHT,
        2000,               //
        {.minSpeed = 30});  // Point towards the goal

    chassis.moveToPoint(16, 60, 1000);  // Move to the goal
    arm.setPosition(190);               // Score preload

    pros::delay(300);  // Make sure Arm gets there first
    rush.extend();     // Extend the rush mechanism
    pros::delay(200);  // Wait for rush to extend

    chassis.moveToPoint(11.5, 24, 2000, {.forwards = false});  // Retreat
    pros::delay(400);
    arm.setPosition(250);  // Fully score the arm
    pros::delay(800);
    arm.setPosition(50);  // Reset Arm
    rush.retract();       // Reset Rush

    // Goal 2
    clamp.enableAutoClamp();
    chassis.turnToPoint(30, 24, 800, {.minSpeed = 30});
    chassis.moveToPoint(30, 24, 1000, {.minSpeed = 10});
    chassis.turnToPoint(48, 48, 1000, {.forwards = false});
    chassis.moveToPoint(46, 48, 1000, {.forwards = false, .maxSpeed = 80});
    intake.enableSort(Intake::RingColors::RED);  // Sort Red rings
    pros::delay(100);
    intake.spin();
    chassis.turnToPoint(24, 46, 1000, {.minSpeed = 20});  // Grab Ring
    chassis.moveToPoint(24, 46, 1000, {.maxSpeed = 110});

    chassis.moveToPoint(48, 48, 1000, {false, 80});
    chassis.turnToPoint(0, 0, 800);
    chassis.moveToPoint(9.369, 9.369, 800, {.minSpeed = 40}, false);
    chassis.moveToPoint(9.369, 9.369, 1000, {.maxSpeed = 40}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 700, {.minSpeed = 80});
    pros::delay(700);

    chassis.setPose(12, 12, chassis.getPose().theta);
    chassis.moveToPoint(24, 24, 800, {.forwards = false, .maxSpeed = 50});
    arm.setPosition(0);  // Reset Arm
    chassis.turnToPoint(60, 60, 2000, {.maxSpeed = 80, .minSpeed = 30});
    chassis.moveToPoint(60, 60, 2000, {}, false);
    chassis.waitUntilDone();
  }
}

void soloWP(bool isRedTeam)
{
  if (isRedTeam) {}
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
    chassis.moveToPoint(93, 24, 250, {.maxSpeed = 70, .minSpeed = 60});
    chassis.moveToPoint(93, 24, 1700, {.maxSpeed = 40, .minSpeed = 10});

    int time = 0;
    while ((intake.getCurrentRingColor() != Intake::RingColors::BLUE) && time < 980)
    {
      time += 10;
      pros::delay(10);
    }
    intake.stop();

    // * Goal 2
    chassis.turnToPoint(99, 48, 1000, {false});
    clamp.enableAutoClamp();
    chassis.moveToPoint(99, 48, 1000, {false, 80});

    // ? Ring 3
    chassis.moveToPoint(123, 49.5, 3000, {.maxSpeed = 80, .minSpeed = 20});
    pros::delay(300);
    intake.spin();
    chassis.waitUntilDone();
    pros::delay(400);

    // * TOUCH BAR
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
    chassis.moveToPoint(-130, 46, 2200, {.maxSpeed = 40});
    intake.enableSort(Intake::RingColors::BLUE);
    intake.spin();
    chassis.waitUntilDone();
    pros::delay(600);
    chassis.moveToPoint(-119, 46, 1000, {.forwards = false});  // Back up

    // * Alliance Stake
    chassis.turnToPoint(-84, 20, 1000, {.maxSpeed = 80, .minSpeed = 10});
    chassis.moveToPoint(-84, 20, 3000, {.maxSpeed = 90});
    arm.setIndex(1);
    pros::delay(1500);
    intake.stop();

    chassis.moveToPoint(-77, 11, 3000, {.minSpeed = 20});
    chassis.turnToPoint(-72.5, 1, 1000, {}, false);

    arm.setPosition(200);
    pros::delay(800);

    // * Touch Bar
    chassis.moveToPoint(-96, 48, 2000, {.forwards = false, .minSpeed = 40});
    pros::delay(200);
    arm.setPosition(0);
    chassis.turnToPoint(-89, 52, 1000, {.minSpeed = 20});
    chassis.moveToPoint(-89, 52, 2000);
  }
  else
  {
    chassis.setPose({97.8, 21.67, 22});  // Set the robot's position
    rush.extend();                       // Extend the rush mechanism
    chassis.moveToPoint(
        chassis.getPose().x + std::sin(chassis.getPose(true).theta) * 41.5,
        chassis.getPose().y + std::cos(chassis.getPose(true).theta) * 41.5,
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
    chassis.turnToPoint(76.3, 1, 1000, {.minSpeed = 5}, false);

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

void debug(bool isRedTeam) { chassis.setPose({72, 24, 0}); }

}  // namespace Autonomous