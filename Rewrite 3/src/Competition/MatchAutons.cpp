#include <sys/types.h>

#include <cmath>

#include "2131H/Systems/Intake.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"

namespace Autonomous
{
void debug(bool redTeam) { chassis.turnToHeading(90, 20000000); }

void goalSide(bool redTeam)
{
  if (redTeam)
  {
    // ARM INITAL
    arm.enable();
    arm.setIndex(1);

    // CHASSIS INITIAL
    chassis.setPose(
        {TILE * 4.0 + DOVE_TAIL - DRIVE_LENGTH_OFFSET - 1,  //
         TILE + DOVE_TAIL - DRIVE_WIDTH_OFFSET,
         -90});

    // ALLIANCE STAKE
    chassis.turnToPoint(TILE * 3.0 + DOVE_TAIL / 2.0 + 1, 2.0, 1000, {.minSpeed = 30}, false);
    arm.setPosition(200);
    chassis.moveLinear(5.5, 2000, {}, false);

    // * GOAL 1
    // NOTE add 19 to goal
    chassis.moveLinear(-8, 2000, {}, false);
    arm.setIndex(0);

    chassis.turnToPoint(
        TILE * 4 + DOVE_TAIL / 2.0,
        TILE * 2 + DOVE_TAIL / 2.0,  //
        1000,
        {.forwards = false, .minSpeed = 30},
        false);

    clamp.enableAutoClamp();

    chassis.moveToPoint(
        TILE * 4 + DOVE_TAIL / 2.0,
        TILE * 2 + DOVE_TAIL / 2.0,  //
        2000,
        {.forwards = false, .maxSpeed = 63.5},
        false);

    // ? Inverted Rings
    intake.lift();
    intake.spin(12000, 1);

    chassis.turnToPoint(
        TILE * 3 + DRIVE_WIDTH_OFFSET * std::sin(M_PI / 4.0),  //
        TILE + DRIVE_LENGTH_OFFSET * std::cos(M_PI / 4.0),
        1000,
        {.minSpeed = 40},
        false);

    chassis.moveToPoint(
        TILE * 3 + (DRIVE_WIDTH_OFFSET)*std::sin(M_PI / 4.0),  //
        TILE + (DRIVE_LENGTH_OFFSET)*std::cos(M_PI / 4.0),
        2000,
        {},
        false);

    intake.drop();

    // ? Normal Ring Stack
    chassis.moveLinear(-6, 2000, {.minSpeed = 30}, false);

    chassis.turnToPoint(
        TILE * 5 - 4,
        TILE * 2 - 3,  //
        1000,
        {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 30},
        false);

    intake.spin(12000);

    chassis.moveToPoint(
        TILE * 5 - 4,
        TILE * 2 - 3,  //
        2000,
        {.minSpeed = 10},
        false);

    // * TO CORNER

    chassis.turnToHeading(135, 1000, {.minSpeed = 100}, false);

    chassis.moveToPose(
        TILE * 5.0,
        TILE - 3.0,  //
        -135,
        1500,
        {},
        true);

    pros::delay(300);
    intake.spin(-12000, 1);

    arm.setPosition(170);

    chassis.turnToHeading(135, 2000, {.minSpeed = 50}, false);

    // CORNER SHENANIGANS
    chassis.moveLinear(26, 1450, {.maxSpeed = 50}, true);

    // Time out-taking
    pros::delay(950);

    intake.spin(12000.0, 1);

    // Time in-taking
    chassis.waitUntilDone();
    pros::delay(500);

    chassis.moveLinear(-4, 2000, {}, false);

    intake.lift();
    intake.spin(-12000.0, 1);

    chassis.moveLinear(5, 500, {}, true);

    pros::delay(300);
    intake.spin(12000.0, 1);
    pros::delay(400);

    chassis.movePolar(-70, 132, 2000, {.maxSpeed = 100}, false, true);
    pros::delay(300);
    arm.setPosition(45);

    intake.drop();
    pros::delay(500);
    intake.stop();
  }
  else
  {
    // ARM INITAL
    arm.enable();
    arm.setIndex(1);

    // CHASSIS INITIAL
    chassis.setPose(
        {-(TILE * 4.0 + DOVE_TAIL - DRIVE_LENGTH_OFFSET - 1),  //
         TILE + DOVE_TAIL - DRIVE_WIDTH_OFFSET,
         90});

    // ALLIANCE STAKE
    chassis.turnToPoint(-(TILE * 3.0 + DOVE_TAIL / 2.0), 2.0, 1000, {.minSpeed = 30}, false);
    arm.setPosition(200);
    chassis.moveLinear(6.8, 2000, {}, false);

    // * GOAL 1
    // NOTE add 19 to goal
    chassis.moveLinear(-8, 2000, {}, false);
    arm.setIndex(0);

    chassis.turnToPoint(
        -(TILE * 4 + DOVE_TAIL / 2.0),
        TILE * 2 + DOVE_TAIL / 2.0,  //
        1000,
        {.forwards = false, .minSpeed = 30},
        false);

    clamp.enableAutoClamp();

    chassis.moveToPoint(
        -(TILE * 4 + DOVE_TAIL / 2.0),
        TILE * 2 + DOVE_TAIL / 2.0,  //
        2000,
        {.forwards = false, .maxSpeed = 63.5},
        false);

    // ? Inverted Rings
    intake.lift();
    intake.spin(12000, 1);

    chassis.turnToPoint(
        -(TILE * 3 + DRIVE_WIDTH_OFFSET * std::sin(M_PI / 4.0)),  //
        TILE + DRIVE_LENGTH_OFFSET * std::cos(M_PI / 4.0),
        1000,
        {.minSpeed = 40},
        false);

    chassis.moveToPoint(
        -(TILE * 3 + (DRIVE_WIDTH_OFFSET + 0.5) * std::sin(M_PI / 4.0)),  //
        TILE + (DRIVE_LENGTH_OFFSET + 0.5) * std::cos(M_PI / 4.0),
        1800,
        {},
        false);

    intake.drop();

    // ? Normal Ring Stack
    chassis.moveLinear(-6, 2000, {.minSpeed = 30}, false);

    chassis.turnToPoint(
        -(TILE * 5 - 4),
        TILE * 2,  //
        1000,
        {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 30},
        false);

    intake.spin(12000);

    chassis.moveToPoint(
        -(TILE * 5 - 4),
        TILE * 2,  //
        2000,
        {.minSpeed = 10},
        false);

    // * TO CORNER

    chassis.turnToHeading(-135, 1000, {.minSpeed = 100}, false);

    chassis.moveToPose(
        -(TILE * 5 + DOVE_TAIL),
        TILE - 2.0,  //
        135,
        1500,
        {},
        true);

    pros::delay(300);
    intake.spin(-12000, 1);

    chassis.turnToHeading(-134, 2000, {.minSpeed = 50}, true);
    pros::delay(500);  // Allow ring to score
    arm.setPosition(170);

    // CORNER SHENANIGANS
    chassis.moveLinear(26, 1450, {.maxSpeed = 50}, true);

    // Time out-taking
    pros::delay(950);

    intake.spin(12000.0, 1);

    // Time in-taking
    chassis.waitUntilDone();
    pros::delay(500);

    chassis.moveLinear(-4, 2000, {}, false);

    intake.lift();
    intake.spin(-12000.0, 1);

    chassis.moveLinear(5, 500, {}, true);

    pros::delay(300);
    intake.spin(12000.0, 1);
    pros::delay(400);

    chassis.movePolar(-70, -132, 2000, {.maxSpeed = 100}, false, true);
    pros::delay(300);
    arm.setPosition(45);

    intake.drop();
    pros::delay(500);
    intake.stop();
  }
}

void ringSide(bool redTeam) { goalSide(!redTeam); }

void soloAWP(bool redTeam)
{
  if (redTeam)
  {
    // ARM INITAL
    arm.enable();
    arm.setIndex(1);

    // CHASSIS INITIAL
    chassis.setPose(
        {TILE * 4.0 + DOVE_TAIL - DRIVE_LENGTH_OFFSET - 1,  //
         TILE + DOVE_TAIL - DRIVE_WIDTH_OFFSET,
         -90});

    // ALLIANCE STAKE
    chassis.turnToPoint(TILE * 3.0 + DOVE_TAIL / 2.0 + 1, 2.0, 1000, {.minSpeed = 30}, false);
    arm.setPosition(200);
    chassis.moveLinear(5.5, 2000, {}, false);

    // * GOAL 1
    // NOTE add 19 to goal
    chassis.moveLinear(-8, 2000, {}, false);
    arm.setIndex(0);

    chassis.turnToPoint(
        TILE * 4 + DOVE_TAIL / 2.0,
        TILE * 2 + DOVE_TAIL / 2.0,  //
        1000,
        {.forwards = false, .minSpeed = 30},
        false);

    clamp.enableAutoClamp();

    chassis.moveToPoint(
        TILE * 4 + DOVE_TAIL / 2.0,
        TILE * 2 + DOVE_TAIL / 2.0,  //
        2000,
        {.forwards = false, .maxSpeed = 70},
        false);

    // ? Normal Stack
    chassis.turnToPoint(TILE * 5.0, TILE * 2.0 + 2.0, 1000, {.minSpeed = 40}, false);
    intake.spin(12000);
    chassis.moveToPoint(TILE * 5.0, TILE * 2.0 + 2.0, 2000, {}, false);
    intake.lift();

    // ? Inverted Stack
    chassis.turnToHeading(180, 1000, {.minSpeed = 50}, false);
    intake.stop();
    intake.spin(12000.0, 1);

    chassis.moveToPose(
        TILE * 3 + DRIVE_LENGTH_OFFSET * 2.0 - 1.00,
        TILE + 2.0,  //
        -90.0,
        3000,
        {.minSpeed = 40},
        false);

    intake.drop();
    clamp.disableAutoClamp();

    // * GOAL 2
    chassis.moveLinear(24, 2000, {}, false);
    chassis.turnToPoint(
        48 + 3,
        48 + 3,  //
        2000.0,
        {.forwards = false, .minSpeed = 30},
        false);
    clamp.enableAutoClamp();
    chassis.moveToPoint(
        48 + 3,
        48 + 3,  //
        2000.0,
        {.forwards = false, .maxSpeed = 60},
        false);

    // ? Normal Stack 2
    arm.setPosition(60);
    chassis.turnToHeading(-102, 1500, {.minSpeed = 30}, false);
    intake.spin(12000);
    chassis.movePolar(24, -102, 2000, {}, false, false);

    // * Ladder
    chassis.moveToPose(72, 66, -135, 2000, {.forwards = false, .maxSpeed = 80}, true);
    pros::delay(300);
    intake.stop();
  }
  else
  {  // ARM INITAL
    arm.enable();
    arm.setIndex(1);

    // CHASSIS INITIAL
    chassis.setPose(
        {-(TILE * 4.0 + DOVE_TAIL - DRIVE_LENGTH_OFFSET),  //
         TILE + DOVE_TAIL - DRIVE_WIDTH_OFFSET,
         90});

    // ALLIANCE STAKE
    chassis.turnToPoint(-(TILE * 3.0 + DOVE_TAIL / 2.0), 2.0, 1000, {.minSpeed = 30}, false);
    arm.setPosition(200);
    chassis.moveLinear(6.5, 2000, {}, false);

    // * GOAL 1
    chassis.moveLinear(-9, 2000, {}, false);
    arm.setIndex(0);

    chassis.turnToPoint(
        -(TILE * 4 + DOVE_TAIL / 2.0),
        TILE * 2 + DOVE_TAIL / 2.0,  //
        1000,
        {.forwards = false, .minSpeed = 30},
        false);

    clamp.enableAutoClamp();

    chassis.moveToPoint(
        -(TILE * 4 + DOVE_TAIL / 2.0),
        TILE * 2 + DOVE_TAIL / 2.0,  //
        2000,
        {.forwards = false, .maxSpeed = 70},
        false);

    // ? Normal Stack
    chassis.turnToPoint(-(TILE * 5.0), TILE * 2.0, 1000, {.minSpeed = 40}, false);
    intake.spin(12000);
    chassis.moveToPoint(-(TILE * 5.0), TILE * 2.0, 2000, {}, false);
    intake.lift();

    // ? Inverted Stack
    chassis.turnToHeading(-180, 1000, {.minSpeed = 50}, false);
    intake.stop();
    intake.spin(12000.0, 1);

    chassis.moveToPose(
        -(TILE * 3 + 3.0),
        TILE,  //
        90.0,
        3000,
        {.minSpeed = 30},
        false);

    intake.drop();
    clamp.disableAutoClamp();

    // * GOAL 2
    chassis.moveLinear(24, 2000, {}, false);
    chassis.turnToPoint(
        -(48 - 2),
        48 + 3,  //
        2000.0,
        {.forwards = false, .minSpeed = 30},
        false);
    clamp.enableAutoClamp();
    chassis.moveToPoint(
        -(48 - 2),
        48 + 3,  //
        2000.0,
        {.forwards = false, .maxSpeed = 60},
        false);

    intake.spin(12000, 3);

    // ? Normal Stack 2
    arm.setPosition(60);
    chassis.turnToHeading(102, 1500, {.minSpeed = 30}, false);
    intake.spin(12000);
    chassis.movePolar(24, 102, 2000, {}, false, false);

    // * Ladder
    chassis.moveToPose(-60, 66, 135, 2000, {.forwards = false, .maxSpeed = 80}, true);
    pros::delay(300);
    intake.stop();
  }
}
}  // namespace Autonomous