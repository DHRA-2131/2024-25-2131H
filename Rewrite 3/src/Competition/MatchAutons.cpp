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
    chassis.moveLinear(7, 2000, {}, false);

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
    intake.spin(12000);

    chassis.turnToPoint(
        TILE * 3 + DRIVE_WIDTH_OFFSET * std::sin(M_PI / 4.0),  //
        TILE + DRIVE_LENGTH_OFFSET * std::cos(M_PI / 4.0),
        1000,
        {.minSpeed = 40},
        false);

    chassis.moveToPoint(
        TILE * 3 + (DRIVE_WIDTH_OFFSET - 1 * std::sin(M_PI / 4.0)),  //
        TILE + (DRIVE_LENGTH_OFFSET - 1 * std::cos(M_PI / 4.0)),
        2000,
        {},
        false);

    // ? Normal Ring Stack
    chassis.moveLinear(-6, 2000, {.minSpeed = 30});

    pros::delay(500);
    intake.drop();

    chassis.turnToPoint(
        TILE * 5 + DOVE_TAIL,
        TILE * 2 + DOVE_TAIL,  //
        1000,
        {.direction = lemlib::AngularDirection::CW_CLOCKWISE},
        false);

    chassis.moveToPoint(
        TILE * 5 + DOVE_TAIL,
        TILE * 2 + DOVE_TAIL,  //
        1000,
        {},
        false);

    // TO CORNER
    intake.spin(-12000, 1);

    chassis.turnToHeading(135, 1000, {.minSpeed = 100}, false);

    chassis.moveToPose(
        TILE * 5.0,
        TILE - 1.5,  //
        -135,
        1500,
        {},
        false);

    arm.setPosition(190);

    chassis.turnToHeading(132, 2000, {.minSpeed = 50}, false);

    // CORNER SHENANIGANS
    chassis.moveLinear(23, 1500, {.maxSpeed = 40}, true);

    pros::delay(950);

    intake.spin(12000.0, 1);
    pros::delay(1000);
    chassis.moveLinear(-4, 2000, {}, false);

    intake.lift();
    intake.spin(-12000.0, 1);

    chassis.moveLinear(5, 1200, {}, true);
    pros::delay(300);
    intake.spin(12000.0, 1);

    arm.setPosition(45);
    chassis.movePolar(-70, 130, 2000, {}, false, true);

    pros::delay(500);
    intake.drop();
  }
}

}  // namespace Autonomous