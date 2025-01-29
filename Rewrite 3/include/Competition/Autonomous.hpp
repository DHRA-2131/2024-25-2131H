#pragma once

namespace Autonomous
{
void skills(bool isRedTeam);

// Solo Autonomous Win Point (AWP)
void soloAWP(bool isRedTeam);

// Safe Autonomous
void safeRing(bool isRedTeam);
void safeGoal(bool isRedTeam);

// Four Ring Autos
void fourRingGoal(bool isRedTeam);
void fourRingRing(bool isRedTeam);

// Finals
void sixRingFinals(bool isRedTeam);
void goalRush(bool isRedTeam);

// Debug Auto
void debug(bool isRedTeam);
}  // namespace Autonomous