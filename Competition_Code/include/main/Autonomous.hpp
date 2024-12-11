#pragma once

#include "RobotConfig.hpp"

namespace Autonomous
{
void low4RG(bool isRedTeam);   // Low stake auton, 4 rings on one goal (Side w/o high stake)
void highStake(bool isRedTeam);// High stake auton (Side w/ high stake )
void goalRush(bool isRedTeam);  //goal rush auton (grab middle goal first)
void lowAlliance(bool isRedTeam);     // Low stake auton, alliance stake & 3 rings on one goal (Side w/ low stake )
void skills(bool isRedTeam);     // Skills auton
void debug(bool isRedTeam);      // Debug auton (not for competition use)
}  // namespace Autonomous