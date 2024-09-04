#include "systems/Drivetrain.hpp"

#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"
#include "pros/misc.h"


namespace Systems
{
namespace Drivetrain
{
void teleOp(pros::Controller& primary)
{
  leftDrive.move(primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 12000.0);
  rightDrive.move(primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0 *
                  12000.0);

  if (Buttons::ClampToggle.changedToPressed()) { clamp.toggle(); }
}
}  // namespace Drivetrain
}  // namespace Systems