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
  chassis.setPose(72, 10.25, 0);  // Set up by alliance stake

  //* Score on alliance
  Intake::motor.move_voltage(12000);
  pros::delay(500);
  Intake::motor.brake();

  //? Grab Goal 1
  chassis.moveToPoint(72, 24, 1000, {}, false);
  chassis.turnToPoint(48, 24, 800, {false}, false);
  chassis.moveToPoint(48, 24, 1000, {false, 80});  // Move backwards slowly

  //* Ring 1
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(48, 48, 1000);

  //* Ring 2 (Less Y bias)
  chassis.moveToPoint(12, 68, 1500);

  //* Ring 3
  chassis.turnToPoint(20, 96, 800, {}, false);
  chassis.moveToPoint(20, 96, 800, {}, false);

  //* Ring 4, 5, 6
  chassis.turnToPoint(26, 48, 1000, {}, false);
  chassis.moveToPoint(26, 48, 2000, {.minSpeed = 100}, false);
  chassis.moveToPoint(24, 10, 2000, {.maxSpeed = 90}, false);

  //? Corner
  chassis.turnToPoint(12, 12, 1000, {false}, false);
  Intake::motor.move_voltage(-12000);
  chassis.moveToPoint(12, 12, 1000, {false}, false);
  Clamp::disableAutoClamp();  // Drop the goal

  //? Goal 2
  chassis.moveToPoint(48, 24, 1000);
  chassis.turnToPoint(72, 24, 400);
  chassis.moveToPoint(72, 24, 1400);

  chassis.turnToPoint(96, 24, 800, {false});
  Clamp::enableAutoClamp();
  chassis.moveToPoint(96, 24, 1000, {false, 90});

  // //! Repeat but flip coords
  // //* Ring 1
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(144 - 48, 48, 1000);

  // //* Ring 2
  chassis.moveToPoint(144 - 18, 72, 1500);  // bias -x

  // //* Ring 3
  chassis.turnToPoint(144 - 26, 96, 800, {}, false);
  chassis.moveToPoint(144 - 26, 96, 800, {}, false);

  // //* Ring 4, 5, 6
  chassis.turnToPoint(144 - 26, 48, 1000, {}, false);
  chassis.moveToPoint(144 - 26, 48, 2000, {.minSpeed = 100}, false);
  chassis.moveToPoint(144 - 26, 10, 2000, {.maxSpeed = 90}, false);

  // //? Corner
  chassis.turnToPoint(144 - 12, 12, 1000, {false}, false);
  Intake::motor.move_voltage(-12000);
  chassis.moveToPoint(144 - 12, 12, 1000, {false}, false);
  Clamp::disableAutoClamp();  // Drop the goal

  while (true) { pros::delay(10); }
}
void debug(bool isRedTeam) { chassis.setPose({0, 0, 0}); }
}  // namespace Autonomous