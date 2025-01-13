#include <cmath>

#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace Autonomous
{
void goalRushFinals(bool isRedTeam)
{
  if (isRedTeam) {}
  else {}
}

void ringSideFinals(bool isRedTeam)
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
    intake.enableSort(Intake::RingColors::BLUE);
    chassis.moveToPoint(50, 49 - 4, 300, {.forwards = false, .maxSpeed = 100, .minSpeed = 30});
    chassis.moveToPoint(50, 49 - 4, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Ring 1 & 2
    chassis.turnToPoint(33, 61.875 - 8, 2000, {.minSpeed = 30});
    chassis.waitUntilDone();
    intake.spin();
    chassis.moveToPoint(33, 61.875 - 8, 3000, {.minSpeed = 75});
    chassis.swingToHeading(-85, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 35});
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x - 12, chassis.getPose().y, 2000);

    // ? Ring 3
    chassis.moveToPoint(36.5 + 3, 45 - 6, 2000, {false});
    chassis.swingToHeading(
        180,
        lemlib::DriveSide::LEFT,
        2000,
        {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 40});

    // * Corner
    chassis.moveToPoint(22, 7, 2500, {.minSpeed = 30});
    chassis.turnToHeading(-135, 2000, {.minSpeed = 20});
    chassis.waitUntilDone();

    // ? Ring 4
    intake.spin(-4000);
    chassis.moveToPoint(
        chassis.getPose().x + sin(chassis.getPose(true).theta) * 10,
        chassis.getPose().y + cos(chassis.getPose(true).theta) * 10,
        900,
        {.maxSpeed = 40});

    chassis.swingToHeading(-120, lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 127});
    pros::delay(300);
    intake.spin(12000);

    pros::delay(500);

    chassis.moveToPoint(
        chassis.getPose().x + sin(chassis.getPose(true).theta) * -5.0,
        chassis.getPose().y + cos(chassis.getPose(true).theta) * -5.0,
        3000,
        {.maxSpeed = 15});

    // ? Ring 5
    chassis.waitUntilDone();

    //! Reckon
    chassis.setPose(5, 14, chassis.getPose().theta);

    chassis.moveToPoint(36, 24, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(-90, 2000, {.minSpeed = 20});
    intake.lift();
    chassis.moveToPoint(56, 24, 2000, {});
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.moveToPoint(40, 36, 2000, {.forwards = false, .maxSpeed = 40});
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
        -46, 30 + 2, 800, {.forwards = false, .maxSpeed = 40, .minSpeed = 30}, false);
    chassis.moveToPoint(-46, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    arm.setIndex(0);
    chassis.turnToPoint(-46, 42, 1000, {.forwards = false, .minSpeed = 30});
    clamp.enableAutoClamp();
    intake.enableSort(Intake::RingColors::RED);
    chassis.moveToPoint(-46, 44, 300, {.forwards = false, .maxSpeed = 100, .minSpeed = 30});
    chassis.moveToPoint(-46, 44, 1000, {.forwards = false, .maxSpeed = 60});

    // ? Ring 1 & 2
    chassis.turnToPoint(-31, 61.875, 2000, {.minSpeed = 30});
    chassis.waitUntilDone();
    intake.spin();
    chassis.moveToPoint(-31, 61.875, 3000, {.minSpeed = 75});
    chassis.swingToHeading(85, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 35});
    chassis.moveToPoint(chassis.getPose().x + 17, chassis.getPose().y, 2000);

    // ? Ring 3
    chassis.moveToPoint(-36.5, 50, 2000, {false});
    chassis.swingToHeading(
        -180,
        lemlib::DriveSide::RIGHT,
        2000,
        {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 40});

    // * Corner
    chassis.moveToPoint(-16, 24, 2500, {.minSpeed = 30});
    chassis.turnToHeading(135, 2000, {.minSpeed = 20});
    chassis.waitUntilDone();

    // ? Ring 4
    intake.spin(-4000);
    chassis.moveToPoint(
        chassis.getPose().x + sin(chassis.getPose(true).theta) * 10,
        chassis.getPose().y + cos(chassis.getPose(true).theta) * 10,
        900,
        {.maxSpeed = 40});

    chassis.swingToHeading(120, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 127});
    pros::delay(300);
    intake.spin(12000);

    pros::delay(500);

    chassis.moveToPoint(
        chassis.getPose().x + sin(chassis.getPose(true).theta) * -5.0,
        chassis.getPose().y + cos(chassis.getPose(true).theta) * -5.0,
        3000,
        {.maxSpeed = 15});

    // ? Ring 5
    chassis.waitUntilDone();

    //! Reckon
    chassis.setPose(-5, 14, chassis.getPose().theta);

    chassis.moveToPoint(-36, 24, 2000, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(-90, 2000, {.minSpeed = 20});
    intake.lift();
    chassis.moveToPoint(-48, 24, 2000, {});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(-40, 36, 2000, {false});
  }
  chassis.waitUntilDone();
}
}  // namespace Autonomous