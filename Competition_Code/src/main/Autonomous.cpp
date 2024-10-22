#include "main/Autonomous.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "main/RobotConfig.hpp"
#include "systems/Clamp.hpp"
#include "systems/Intake.hpp"

namespace Autonomous
{
using namespace Systems;

void lowStake(bool isRedTeam)
{
  if (isRedTeam)
  {
    chassis.setPose(48 - 7.25, 24 - 14, 180);       // Start Pose
    chassis.moveToPoint(41.75, 28, 1000, {false});  // Stage Goal
    Clamp::enableAutoClamp();                       // Enable clamp
    chassis.swingToHeading(210, lemlib::DriveSide::LEFT, 800,
                           {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 50});  // Turn to goal
    chassis.moveToPoint(50, 50, 1000, {false}, false);
    chassis.turnToPoint(15, 48, 800);                       // Turn to ring1
    chassis.moveToPoint(15, 48, 2000, {.maxSpeed = 63.5});  // Score ring 1
    pros::delay(100);                                       // Don't hit ladder with ring
    Intake::motor.move_voltage(12000);                      // Score one preload
    chassis.turnToPoint(15, 66, 800);                       // Stage Ring Section (Less X)
    chassis.moveToPoint(18, 60, 1500, {.maxSpeed = 40});    // Ring 2 (Hit slow to not cross)
    chassis.moveToPoint(24, 46, 1800, {false});             // Stage Risky Part
    chassis.turnToPoint(24, 72, 1000);                      // Turn to ring 3
    chassis.moveToPoint(24, 60, 1600, {.maxSpeed = 60});    // Ring 3
    chassis.moveToPoint(24, 40, 1000, {false});             // Touch bar
    Intake::motor.brake();                                  // Stop Intake
    chassis.turnToPoint(57, 55, 800);
    chassis.moveToPoint(57, 55, 3000, {}, false);  // Touche bar
  }
  else
  {
    chassis.setPose(144 - 41.75, 10, 180);                // Start Pose
    chassis.moveToPoint(144 - 41.75, 28, 1000, {false});  // Stage Goal
    Clamp::enableAutoClamp();                             // Enable clamp
    chassis.swingToHeading(150, lemlib::DriveSide::RIGHT, 800,
                           {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 50});  // Turn to goal
    chassis.moveToPoint(144 - 50, 50, 1000, {false}, false);
    chassis.turnToPoint(144 - 16, 48, 800);                       // Turn to ring1
    chassis.moveToPoint(144 - 16, 48, 2000, {.maxSpeed = 63.5});  // Score ring 1
    pros::delay(100);                                             // Don't hit ladder with ring
    Intake::motor.move_voltage(12000);                            // Score one preload
    chassis.turnToPoint(144 - 20, 66, 800);                       // Stage Ring Section (Less X)
    chassis.moveToPoint(144 - 23, 60, 1500, {.maxSpeed = 40});    // Ring 2 (Hit slow to not cross)
    chassis.moveToPoint(144 - 36, 46, 1800, {false});             // Stage Risky Part
    chassis.turnToPoint(144 - 36, 72, 1000);                      // Turn to ring 3
    chassis.moveToPoint(144 - 36, 60, 1600, {.maxSpeed = 60});    // Ring 3
    chassis.moveToPoint(144 - 36, 30, 1000, {false});             // Touch bar
    Intake::motor.brake();                                        // Stop Intake
    chassis.turnToPoint(144 - 64, 55, 800);
    chassis.moveToPoint(144 - 64, 55, 3000, {}, false);  // Touche bar
  }
}
void highStake(bool isRedTeam) {}
void skills(bool isRedTeam)
{
  Intake::disableAutoSort();
  // Start Pos
  chassis.setPose({72, 10, 0});
  // Score Alliance stake
  Intake::motor.move_velocity(12000);
  pros::delay(500);
  Intake::motor.brake();
  // Grab Mobile Goal
  chassis.moveToPoint(72, 24, 800);  // Under Shooting Y-Coord works better
  chassis.turnToPoint(96, 20, 500, {false});
  Clamp::enableAutoClamp();
  chassis.moveToPoint(98, 20, 1200, {false, 100}, false);
  // Grab Rings
  Intake::motor.move_voltage(12000);
  chassis.turnToPoint(96, 48, 1000);  // Ring 1
  chassis.moveToPoint(96, 48, 1000);
  chassis.turnToPoint(120, 48, 1000);  // Ring 2
  chassis.moveToPoint(120, 48, 1000);

  chassis.swingToPoint(124, 72, lemlib::DriveSide::LEFT, 800);  // Under Shoot X (Don't hit the wall stake)
  chassis.moveToPoint(124, 72, 1000, {});                       // Ring 3

  chassis.moveToPoint(118, 100, 1000, {}, false);  // Ring 4

  chassis.turnToPoint(120, 10, 800);  // Ring 5 & 6
  chassis.moveToPoint(120, 10, 2100, {.maxSpeed = 100}, false);

  // auto currentPose = chassis.getPose();  // Reckon
  // currentPose.y = 10;
  // chassis.setPose(currentPose);

  chassis.turnToPoint(135, 6, 1000, {false});  // Score Goal
  chassis.moveToPoint(135, 6, 1000, {false}, false);
  Intake::motor.move_voltage(-12000);
  pros::delay(300);  // Allow rings to go on the goal

  Clamp::disableAutoClamp();

  lemlib::Pose currentPose(132, 10, chassis.getPose().theta);  // Reckon
  chassis.setPose(currentPose);

  chassis.moveToPoint(72, 22, 1000);  // Move to center, Undershoot y
  Intake::motor.brake();
  chassis.turnToPoint(48, 22, 800, {false});

  Clamp::enableAutoClamp();
  chassis.moveToPoint(48, 20, 1000, {false, 60});  // Goal 2

  chassis.turnToPoint(48, 48, 800);  // Ring 1
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(48, 48, 1000);

  chassis.turnToPoint(24, 44, 800);  // Ring 2, Under shoot Y
  chassis.moveToPoint(24, 44, 1000);

  chassis.swingToPoint(17, 72, lemlib::DriveSide::RIGHT, 1000);  // Ring 3
  chassis.moveToPoint(17, 72, 1000);                             // Overshoot X Coord to not hit wall stake

  chassis.moveToPoint(34, 96, 1000);  // Ring 4, Overshoot X Coord (Lots)

  chassis.turnToPoint(34, 4, 800);  // Ring 5, 6
  chassis.moveToPoint(34, 4, 2000, {true, 100}, false);

  // auto currentPose = chassis.getPose();  // Reckon
  // currentPose.y = 10;
  // chassis.setPose(currentPose);
  // pros::delay(500);  // Allow rings to go on the goal

  chassis.turnToPoint(0, 6, 800, {false});  // Corner 2
  chassis.moveToPoint(0, 6, 500, {false}, false);
  Intake::motor.move_voltage(-12000);
  pros::delay(300);  // Allow rings to go on the goal
  Clamp::disableAutoClamp();

  chassis.swingToPoint(24, 96, lemlib::DriveSide::LEFT, 1000, {}, false);  // Stage to Goal 3 (Swing to not push goal into the wall)
  chassis.moveToPoint(24, 96, 3000, {}, false);                            // Long movement (needs time)
}
void debug(bool isRedTeam) { chassis.setPose({0, 0, 0}); }
}  // namespace Autonomous