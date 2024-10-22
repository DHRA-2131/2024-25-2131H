#pragma once

#include "RobotConfig.hpp"

namespace Autonomous
{
void lowStake(bool isRedTeam);   // Low stake auton (Side w/o high stake)
void highStake(bool isRedTeam);  // High stake auton (Side w/ high stake)
void skills(bool isRedTeam);     // Skills auton
void debug(bool isRedTeam);      // Debug auton (not for competition use)
}  // namespace Autonomous