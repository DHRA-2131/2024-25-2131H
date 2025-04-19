#pragma once

#define TILE 23.75
#define DOVE_TAIL 13.0 / 16.0
#define DRIVE_LENGTH_OFFSET 13.5 / 2.0
#define DRIVE_WIDTH_OFFSET 15.5 / 2.0
#define RING 7

namespace Autonomous
{
void debug(bool redTeam);
void goalSide(bool redTeam);
}  // namespace Autonomous