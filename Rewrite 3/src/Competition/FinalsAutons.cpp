#include <cmath>

#include "2131H/Systems/Intake.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

namespace Autonomous
{
void ringSideFive(bool redTeam)
{
  if (redTeam)
  {
    // Inital
    chassis.setPose({TILE * 2.0 + DRIVE_LENGTH_OFFSET, TILE + DOVE_TAIL - DRIVE_WIDTH_OFFSET, 90});
    arm.enable();
    arm.setIndex(1);

    // Alliance Stake
    chassis.turnToPoint(TILE * 3.0 + DOVE_TAIL / 2.0, 2, 2000, {.minSpeed = 30}, false);
    arm.setPosition(190);
    chassis.moveLinear(6.25, 2000, {}, false);

    // * Goal 1
    chassis.moveLinear(-9, 1500, {}, false);
    arm.setIndex(0);
    chassis.turnToPoint(48, 48, 2000, {.forwards = false, .minSpeed = 30}, false);
    clamp.enableAutoClamp();
    chassis.moveToPoint(48, 48, 2000, {.forwards = false, .maxSpeed = 80}, false);

    // ? Ladder Rings
    chassis.turnToPoint(
        72 - 3.0 - sin(M_PI_4) * (DRIVE_LENGTH_OFFSET - 1) * 2.0,  //
        72 + 3.5 - 4.0 - cos(M_PI_4) * (DRIVE_LENGTH_OFFSET - 1) * 2.0,
        1000,
        {.minSpeed = 30},
        false);

    doinklerLeft.extend();
    chassis.moveToPoint(
        72 - 3.0 - sin(M_PI_4) * (DRIVE_LENGTH_OFFSET - 1) * 2.0,  //
        72 + 3.5 - 4.0 - cos(M_PI_4) * (DRIVE_LENGTH_OFFSET - 1) * 2.0,
        2000,
        {},
        false);

    // Ladder Retreat
    chassis.moveToPoint(52, 43, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 20}, false);

    // ? Normal Ring
    chassis.turnToHeading(-25, 1000, {.minSpeed = 80}, false);
    doinklerLeft.retract();
    chassis.turnToHeading(-60, 1000, {.minSpeed = 50}, false);

    intake.spin(12000, 3);
    chassis.moveToPoint(24, 48, 2000, {.maxSpeed = 80, .minSpeed = 30}, false);

    // * Corner
    chassis.turnToHeading(180, 1500, {.minSpeed = 50}, false);
    chassis.moveToPose(14, 14, -134, 1500, {.maxSpeed = 80}, true);
    pros::delay(800);  // Delay to Pick up 2nd Ring

    // Push up First Ring
    intake.spin(-12000, 1);
    chassis.movePolar(12, -132, 300, {.minSpeed = 80}, false, false);

    // Grab 1 Corner Ring
    intake.spin(12000, 3);
    pros::delay(100);
    chassis.moveLinear(-3.5, 1500, {}, false);

    // Grab 2 Corner Ring
    intake.lift();
    intake.spin(-12000, 1);
    chassis.moveLinear(7, 500, {}, false);
    intake.spin(12000, 3);
    pros::delay(300);

    // Retreat
    intake.drop();
    chassis.moveLinear(-14, 2000, {}, false);

    // ? Ring 5
    chassis.turnToHeading(90, 1000, {}, false);
    intake.lift();
    chassis.movePolar(40.7, 90, 2000, {}, false, false);
    intake.drop();
    pros::delay(200);
    chassis.moveLinear(-4, 1000, {}, false);  // Wiggle Blue Ring
  }
  else
  {
    // Inital
    chassis.setPose(
        {-(TILE * 2.0 + DRIVE_LENGTH_OFFSET), TILE + DOVE_TAIL - DRIVE_WIDTH_OFFSET, -90});
    arm.enable();
    arm.setIndex(1);

    // Alliance Stake
    chassis.turnToPoint(-(TILE * 3.0 + DOVE_TAIL / 2.0), 1, 2000, {.minSpeed = 30}, false);
    arm.setPosition(190);
    chassis.moveLinear(5.35, 2000, {}, false);

    // * Goal 1
    chassis.moveLinear(-9, 1500, {}, false);
    arm.setIndex(0);
    chassis.turnToPoint(-(48), 48, 2000, {.forwards = false, .minSpeed = 30}, false);
    clamp.enableAutoClamp();
    chassis.moveToPoint(-(48), 48, 2000, {.forwards = false, .maxSpeed = 80}, false);

    // ? Ladder Rings
    chassis.turnToPoint(
        -(72 + 3.5 + 4 - sin(M_PI_4) * (DRIVE_LENGTH_OFFSET + 8.5) * 2.0),
        72 + 3.5 + 3 - cos(M_PI_4) * (DRIVE_LENGTH_OFFSET + 8.5) * 2.0,
        1000,  //! I UH, Forgot to negate the Angle... So remember that
        {.minSpeed = 30},
        false);

    doinklerRight.extend();
    chassis.moveToPoint(
        -(72 + 3.5 + 4 - sin(M_PI_4) * (DRIVE_LENGTH_OFFSET + 8) * 2.0),  //
        72 + 3.5 + 3 - cos(M_PI_4) * (DRIVE_LENGTH_OFFSET + 8) * 2.0,
        2000,
        {.minSpeed = 5},
        false);

    // Ladder Retreat
    chassis.moveToPoint(
        -(52), 43, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 20}, false);

    // ? Normal Ring
    chassis.turnToHeading(35, 1000, {.minSpeed = 80}, false);
    doinklerRight.retract();
    chassis.turnToHeading(60, 1000, {.minSpeed = 50}, false);

    intake.spin(12000, 3);
    chassis.moveToPoint(-(24), 48, 2000, {.maxSpeed = 80, .minSpeed = 30}, true);
    pros::delay(800);  // Delay to Pick up 2nd Ring
    intake.stop();     // Stop Rings from throwing

    // * Corner
    chassis.turnToHeading(-180, 1500, {.minSpeed = 50}, false);
    intake.spin(12000, 3);  // Score rings
    chassis.moveToPose(-(9.5), 14, 133, 1500, {.lead = 0.3, .maxSpeed = 80}, true);
    pros::delay(800);  // Delay to Pick up 2nd Ring

    // Push up First Ring
    intake.spin(-12000, 1);
    chassis.movePolar(12, 135, 500, {.maxSpeed = 60}, false, false);

    // Grab 1 Corner Ring
    intake.spin(12000, 3);
    pros::delay(300);
    chassis.moveLinear(-3.5, 1500, {.maxSpeed = 70}, false);

    // Grab 2 Corner Ring
    intake.lift();
    intake.spin(-12000, 1);
    chassis.moveLinear(7, 500, {}, false);
    intake.spin(12000, 3);
    pros::delay(300);

    // Retreat
    intake.drop();
    chassis.moveLinear(-14.5, 2000, {}, false);

    // ? Ring 5
    chassis.turnToHeading(-86, 1000, {.minSpeed = 30}, false);
    intake.lift();
    chassis.movePolar(41.5, -90, 2000, {.minSpeed = 10}, false, false);
    intake.drop();
    chassis.moveLinear(-4, 1000, {}, false);  // Wiggle Blue Ring
  }
}

void goalSideFive(bool redTeam) {}

void goalRush(bool redTeam)
{
  if (redTeam)
  {
    // Inital
    arm.enable();
    chassis.setPose({TILE * 5.0 - 14.0, TILE - 3.0, 25.5});
    doinklerLeft.extend();

    // Goal 1
    chassis.moveLinear(30.5, 2000, {}, true);
    pros::delay(750);       // Allows for doinkler to retract before retreat
    intake.spin(12000, 1);  // Store Annoying Ring

    // Retreat
    doinklerLeft.retract();
    chassis.waitUntilDone();                                              // * CHASSIS WAIT
    chassis.movePolar(-4, 25.5, 15000, {.minSpeed = 127}, false, false);  // TIMEOUT SPEEDY MOTION
    chassis.movePolar(
        -16,
        28,
        2000,
        {.minSpeed = 10},  // (Min Speed to Let goal release)
        false,
        true);         // CONTINUE PATH
    pros::delay(200);  // Gets Drag Distance
    doinklerLeft.extend();
    pros::delay(800);  // Drop Goal

    // Goal 2
    chassis.turnToHeading(160, 1000, {.minSpeed = 30}, false);  // Bat Ring
    chassis.turnToPoint(TILE * 4.0, TILE * 2.0, 2000, {.forwards = false, .minSpeed = 10}, false);
    clamp.enableAutoClamp();
    chassis.moveToPoint(TILE * 4.0, TILE * 2.0, 2000, {.forwards = false, .maxSpeed = 60}, false);
    intake.spin(12000, 3);  // Score Rings
    doinklerLeft.retract();

    pros::delay(200);

    chassis.turnToPoint(144 - 3.5, 3.5 + 0.5, 2000, {.minSpeed = 5}, false);  // Turn to Corner

    // Drop Goal
    intake.spin(-12000, 1);  // Corner Intake Shenanigans
    chassis.moveLinear(24, 2000, {.minSpeed = 40}, false);
    clamp.disableAutoClamp();

    // Corner
    chassis.moveToPoint(144 - 3.5, 3.5 + 0.5, 1500, {.maxSpeed = 60}, true);
    pros::delay(1000);      // Allow Reversing to get on rings
    intake.spin(12000, 3);  // Corner Stack
    arm.setIndex(1);
  }
}

}  // namespace Autonomous