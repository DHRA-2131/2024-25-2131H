#include "main/ButtonConfig.hpp"

#include "pros/misc.h"

namespace Buttons
{
ButtonDetector Intake(pros::E_CONTROLLER_DIGITAL_L1);   // Intake
ButtonDetector Outtake(pros::E_CONTROLLER_DIGITAL_L2);  // Outtake

ButtonDetector ArmUp(pros::E_CONTROLLER_DIGITAL_R1);    // Arm Up
ButtonDetector ArmDown(pros::E_CONTROLLER_DIGITAL_R2);  // Arm Down
ButtonDetector Doinkler(pros::E_CONTROLLER_DIGITAL_B);  // Doinkler

ButtonDetector ClampToggle(pros::E_CONTROLLER_DIGITAL_X);  // Clamp

/**
 * @brief Update all Buttons
 *
 */
void update()
{
  Intake.update();
  Outtake.update();
  ArmUp.update();
  ArmDown.update();
  Doinkler.update();
  ClampToggle.update();
}
}  // namespace Buttons