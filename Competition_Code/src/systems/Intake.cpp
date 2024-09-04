#include "systems/Intake.hpp"

#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"


namespace Systems
{
namespace Intake
{
void teleOp(pros::Controller& primary)
{
  if (Buttons::Intake.isPressing()) { intake.move_voltage(12000); }
  else if (Buttons::Outtake.isPressing()) { intake.move_voltage(-12000); }
  else { intake.brake(); }
}
}  // namespace Intake
}  // namespace Systems