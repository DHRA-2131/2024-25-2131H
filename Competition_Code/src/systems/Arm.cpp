#include "systems/Arm.hpp"

#include "main/RobotConfig.hpp"
#include "pros/abstract_motor.hpp"

namespace Systems
{
namespace Arm
{

void init()
{
  arm.set_brake_mode(pros::MotorBrake::hold);
  arm.tare_position();
}

void teleOp() {}
}  // namespace Arm
}  // namespace Systems