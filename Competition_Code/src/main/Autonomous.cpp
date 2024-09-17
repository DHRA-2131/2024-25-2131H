#include "main/Autonomous.hpp"

namespace Autonomous
{
void left() {}
void right() {}
void skills() {}
void debug()
{
  chassis.setPose({0, 0, 0});
  intake.move_voltage(12000);
  chassis.moveToPoint(0, 24, 1000);
}
}  // namespace Autonomous