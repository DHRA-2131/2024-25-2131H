#include "systems/Drivetrain.hpp"

#include "main/RobotConfig.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

namespace Systems
{
namespace Drivetrain
{
void teleOp()
{
  leftDrive.move_voltage(primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 12000.0);
  rightDrive.move_voltage(primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0 * 12000.0);
}
}  // namespace Drivetrain
}  // namespace Systems