#pragma once

namespace Autonomous
{
void soloWP(bool isRedTeam);          // High stake auton
void goalRush(bool isRedTeam);        // Low stake auton
void safeGoalSide(bool isRedTeam);    // Safe goal side auton
void goalRushFinals(bool isRedTeam);  // Low stake auton  (Finals)
void ringRush(bool isRedTeam);        // goal rush auton
void safeRingSide(bool isRedTeam);    // Safe ring side auton
void ringRushFinals(bool isRedTeam);  // goal rush auton (Finals)
void skills(bool isRedTeam);          // Skills auton
void debug(bool isRedTeam);           // Debug auton (not for competition use)
}  // namespace Autonomous