#pragma once

#include "ButtonDetector.hpp"
#include "main/RobotConfig.hpp"

namespace Buttons
{
extern ButtonDetector Intake;   // Intake
extern ButtonDetector Outtake;  // Outtake

extern ButtonDetector ArmUp;    // Arm Up
extern ButtonDetector ArmDown;  // Arm Down

extern ButtonDetector ClampToggle;  // Clamp

void update();  // Update all buttons
}  // namespace Buttons