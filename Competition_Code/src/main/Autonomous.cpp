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
    chassis.moveToPoint(41.75, 27, 1000, {false});  // Stage Goal
    Clamp::enableAutoClamp();                       // Enable clamp
    chassis.swingToHeading(210, lemlib::DriveSide::LEFT, 800,
                           {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 50});  // Turn to goal
    chassis.moveToPoint(50, 50, 1000, {false}, false);
    chassis.turnToPoint(22, 44, 800);                      // Turn to ring1
    chassis.moveToPoint(22, 44, 2000, {.maxSpeed = 100});  // Score ring 1
    pros::delay(100);                                      // Don't hit ladder with ring
    Intake::motor.move_voltage(12000);                     // Score one preload
    chassis.turnToPoint(10, 66, 800);                      // Stage Ring Section (Less X)
    chassis.moveToPoint(18, 60, 1500, {.maxSpeed = 40});   // Ring 2 (Hit slow to not cross)
    // chassis.moveToPoint(24, 46, 1800, {false});            // Stage Risky Part
    // chassis.turnToPoint(24, 72, 1000);                     // Turn to ring 3
    // chassis.moveToPoint(24, 60, 1600, {.maxSpeed = 60});   // Ring 3
    chassis.moveToPoint(24, 40, 1000, {false});  // Touch bar

    chassis.waitUntilDone();
  }
  else
  {
    //? Goal 1
    chassis.setPose(144 - 41.75, 17, 180);                  // Start Pose
    chassis.moveToPoint(144 - 41.75, 34.5, 1000, {false});  // Stage Goal
    Clamp::enableAutoClamp();                               // Enable clamp
    chassis.swingToHeading(150, lemlib::DriveSide::RIGHT, 800,
                           {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 50});  // Turn to goal
    chassis.moveToPoint(144 - 53, 53, 1000, {false}, false);

    //* Ring 1 2
    chassis.turnToPoint(144 - 16, 48, 800);                     // Turn to ring
    chassis.moveToPoint(144 - 16, 48, 800, {.maxSpeed = 100});  // Score ring 1
    pros::delay(100);                                           // Don't hit ladder with ring

    //* Ring 3
    Intake::motor.move_voltage(12000);                           // Score one preload
    chassis.turnToPoint(144 - 18, 68, 800);                      // Stage Ring Section (Less X)
    chassis.moveToPoint(144 - 25, 68, 1500, {.maxSpeed = 100});  // Ring 2 (Hit slow to not cross)

    //* Ring 4
    chassis.moveToPoint(144 - 36, 46, 1800, {false, 80});        // Stage Risky Part
    chassis.turnToPoint(144 - 38, 72, 1000);                     // Turn to ring 3
    chassis.moveToPoint(144 - 38, 68, 1600, {.maxSpeed = 100});  // Ring 3

    //? Bar
    chassis.moveToPoint(144 - 36, 30, 1000, {false});
    chassis.turnToPoint(144 - 68, 57, 800, {}, false);
    Intake::motor.brake();                               // Stop Intake
    chassis.moveToPoint(144 - 68, 57, 3000, {}, false);  // Touche bar

    chassis.waitUntilDone();  // Prevent pre mature end
  }
}
void soloWP(bool isRedTeam)
{
  if (isRedTeam)
  {
    Intake::disableAutoSort();
    chassis.setPose(58, 18, -90);

    //? Wall stake
    chassis.moveToPoint(144 - 72, 18, 1200, {false, 70}, false);
    chassis.turnToHeading(-2, 1000, {}, false);
    chassis.moveToPoint(144 - 72, 12, 2000, {false}, false);

    Intake::motor.move_voltage(12000);
    pros::delay(500);
    Intake::motor.move_voltage(-12000);

    //? Goal 1
    chassis.moveToPoint(144 - 72, 16, 800, {}, false);
    Clamp::enableAutoClamp();
    chassis.turnToPoint(144 - 98, 48, 1200, {false}, false);  // Bias -x
    chassis.moveToPoint(144 - 98, 48, 500, {false, 127, 40}, false);
    chassis.moveToPoint(144 - 96, 48, 1000, {false, 40}, false);
    Intake::motor.move_voltage(12000);

    //* Ring 1
    chassis.moveToPoint(144 - 120, 48, 2000);

    //* Ring 2
    chassis.turnToPoint(144 - 132, 66, 1000);
    chassis.moveToPoint(144 - 127.5, 66, 2000, {.maxSpeed = 80});
    pros::delay(500);
    Intake::motor.move_voltage(-12000);

    //? Touch Hang
    chassis.moveToPoint(20, 40, 800, {false});
    chassis.moveToPoint(60, 64, 3000, {false, 80}, false);
    chassis.waitUntilDone();
  }
  else
  {
    Intake::enableAutoSort();
    chassis.setPose(86, 18, 90);

    //? Wall stake
    chassis.moveToPoint(72, 18, 1200, {false, 70}, false);
    chassis.turnToHeading(2, 1000, {}, false);
    chassis.moveToPoint(71, 10.5, 2000, {false}, false);

    Intake::motor.move_voltage(12000);
    pros::delay(500);
    Intake::motor.move_voltage(-12000);

    //? Goal 1
    chassis.moveToPoint(72, 16, 800, {}, false);
    Clamp::enableAutoClamp();
    chassis.turnToPoint(105, 50, 1200, {false}, false);  // Bias +x
    chassis.moveToPoint(105, 50, 500, {false, 127, 40}, false);
    chassis.moveToPoint(105, 50, 1000, {false, 40}, false);

    //* Ring 1
    Intake::motor.move_voltage(12000);
    chassis.moveToPoint(120, 48, 2000);

    //* Ring 3
    chassis.turnToPoint(132, 58, 1000);  // Bias +x
    chassis.moveToPoint(120, 58, 3000, {.maxSpeed = 50}, false);

    //? Touch Hang
    chassis.moveToPoint(120, 40, 800, {false});
    chassis.moveToPoint(86, 60, 3000, {false});
    pros::delay(500);
    Intake::motor.move_voltage(-12000);
    chassis.waitUntilDone();
  }
}

void highStake(bool isRedTeam)
{
  if (isRedTeam)
  {
    //? Goal 1
    chassis.setPose(144 - 41.75, 10, 180);                  // Start Pose
    chassis.moveToPoint(144 - 41.75, 27.5, 1000, {false});  // Stage Goal
    Clamp::enableAutoClamp();                               // Enable clamp
    chassis.swingToHeading(150, lemlib::DriveSide::RIGHT, 800,
                           {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 50});  // Turn to goal
    chassis.moveToPoint(144 - 50, 50, 1000, {false}, false);

    //* Ring 1
    chassis.turnToPoint(144 - 20, 48, 800);
    pros::delay(750);  // Don't hit goal
    Intake::motor.move_voltage(12000);
    chassis.moveToPoint(144 - 20, 48, 900, {.maxSpeed = 100});  // Score ring 1

    //? Corner
    chassis.moveToPoint(144 - 20, 24, 1200, {false}, false);
    Intake::motor.brake();  // Stop Intake
    chassis.swingToPoint(144, -10, lemlib::DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
    Arm::doinkler.extend();  // Clear Corner

    pros::delay(200);
    chassis.moveToPoint(144 - 20, 12, 800, {.maxSpeed = 80}, false);
    chassis.swingToPoint(144, 60, lemlib::DriveSide::RIGHT, 1000,
                         {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 50, .minSpeed = 50}, false);
    Intake::motor.brake();
    Arm::doinkler.retract();
    chassis.turnToPoint(144 - 70, 56, 1000, {.minSpeed = 30}, false);
    chassis.moveToPoint(144 - 70, 56, 3000, {});
    pros::delay(500);
    Clamp::disableAutoClamp();
    chassis.waitUntilDone();
  }

  else
  {
    //? Goal 1
    chassis.setPose(48 - 7.25, 17, 180);              // Start Pose
    chassis.moveToPoint(41.75, 34.5, 1000, {false});  // Stage Goal
    Clamp::enableAutoClamp();                         // Enable clamp
    chassis.swingToHeading(215, lemlib::DriveSide::LEFT, 800,
                           {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 50});  // Turn to goal
    chassis.moveToPoint(48, 48, 1000, {false}, false);

    //* Ring 1
    chassis.turnToPoint(20, 48, 800);                     // Turn to ring1
    chassis.moveToPoint(20, 48, 800, {.maxSpeed = 100});  // Score ring 1
    pros::delay(200);                                     // Don't hit ladder with ring
    Intake::motor.move_voltage(12000);                    // Score one preload

    //? Corner 1
    chassis.turnToPoint(12.5, 24, 1000, {}, false);
    chassis.moveToPoint(12.5, 24, 1000, {}, false);
    Arm::doinkler.extend();
    Intake::motor.brake();
    chassis.moveToPoint(12.5, 13, 1000, {.maxSpeed = 80}, false);
    Clamp::disableAutoClamp();
    chassis.swingToPoint(80, 0, lemlib::DriveSide::RIGHT, 2000, {.maxSpeed = 60, .minSpeed = 60}, false);
    Arm::doinkler.retract();
    chassis.turnToPoint(24, 40, 1000, {.minSpeed = 40});
    chassis.turnToPoint(48, 60, 1000);
    chassis.moveToPoint(48, 60, 800);

    chassis.waitUntilDone();
  }
}

void skills(bool isRedTeam)
{
  chassis.setPose(72, 10.25, 0);  // Set up by alliance stake

  //* Score on alliance
  Intake::motor.move_voltage(12000);
  pros::delay(500);
  Intake::motor.move_voltage(-12000);

  //? Grab Goal 1
  chassis.moveToPoint(72, 24, 1000, {}, false);
  chassis.turnToPoint(48, 24, 800, {false}, false);
  chassis.moveToPoint(48, 24, 1000, {false, 80});  // Move backwards slowly

  //* Ring 1
  chassis.moveToPoint(48, 48, 1000);
  pros::delay(500);
  Intake::motor.move_voltage(12000);

  //* Ring 2 (Less Y bias)
  chassis.moveToPoint(12, 68, 1500);

  //* Ring 3
  chassis.turnToPoint(20, 96, 800, {}, false);
  chassis.moveToPoint(20, 96, 800, {}, false);

  //* Ring 4, 5, 6
  chassis.turnToPoint(26, 48, 1000, {}, false);
  chassis.moveToPoint(26, 48, 2000, {.minSpeed = 100}, false);
  chassis.moveToPoint(24, 10, 2000, {.maxSpeed = 63.5}, false);

  //? Corner
  chassis.turnToPoint(12, 12, 1000, {false}, false);
  Intake::motor.move_voltage(-12000);
  chassis.moveToPoint(12, 12, 1000, {false}, false);
  Clamp::disableAutoClamp();  // Drop the goal

  //! Reckon
  chassis.setPose(18, 8, chassis.getPose().theta);

  //? Goal 2
  chassis.moveToPoint(48, 24, 1000);
  chassis.turnToPoint(72, 22, 400);
  chassis.moveToPoint(72, 22, 1000);

  chassis.turnToPoint(96, 22, 800, {false});
  Clamp::enableAutoClamp();
  chassis.moveToPoint(96, 22, 1000, {false, 90});

  // //! Repeat but flip coords
  // //* Ring 1
  chassis.moveToPoint(144 - 48, 48, 1000);
  pros::delay(500);
  Intake::motor.move_voltage(12000);

  // //* Ring 2
  chassis.moveToPoint(144 - 14, 72, 1500);  // bias -x

  // //* Ring 3
  chassis.turnToPoint(144 - 22, 96, 800, {}, false);
  chassis.moveToPoint(144 - 22, 96, 800, {}, false);

  // //* Ring 4, 5, 6
  chassis.turnToPoint(144 - 20, 48, 1000, {}, false);
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(144 - 20, 48, 2000, {.minSpeed = 100}, false);
  chassis.moveToPoint(144 - 22, 10, 2000, {.maxSpeed = 63.5}, false);

  // //? Corner
  chassis.turnToPoint(144 - 12, 6, 1000, {false}, false);
  Intake::motor.move_voltage(-12000);
  chassis.moveToPoint(144 - 12, 6, 1000, {false}, false);
  Clamp::disableAutoClamp();  // Drop the goa

  //? Goal 3
  chassis.moveToPoint(144 - 28, 96, 3000);
  chassis.turnToPoint(144 - 48, 120, 800, {false});
  chassis.moveToPoint(144 - 48, 120, 2000, {false});
  chassis.turnToPoint(72, 120, 800, {false});
  Clamp::enableAutoClamp();                           // Enable Auto Clamp
  chassis.moveToPoint(72, 120, 2000, {false, 62.5});  // Grab Goal

  //* Rings 1, 2
  chassis.setPose(72, 121.528, 90.4379);
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(144 - 36, 120, 1000, {.minSpeed = 20});  // Approach Fast
  chassis.moveToPoint(144 - 12, 120, 2000, {.maxSpeed = 30});  // Approach Slow

  //? Corner 3
  chassis.turnToPoint(144 - 18, 132, 800, {false}, false);  // Turn to Corner
  Intake::motor.brake();                                    // Stop Intake
  Clamp::disableAutoClamp();                                // Drop Goal
  chassis.moveToPoint(144 - 18, 132, 2000, {false, 30});    // Score Corner

  //? Corner 4
  chassis.moveToPoint(72, 125, 2000);  // Bias -x
  chassis.turnToPoint(48, 125, 800, {false});
  chassis.moveToPoint(0, 144, 1300, {false, 127, 100});

  //* Blue Alliance Stake
  chassis.moveToPoint(72, 120, 4000);

  while (true) { pros::delay(10); }  // Don't let skills exit
}
void debug(bool isRedTeam) { chassis.setPose({0, 0, 0}); }
}  // namespace Autonomous