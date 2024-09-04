#include "systems/Arm.hpp"

#include "main/ButtonConfig.hpp"
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

void teleOp(pros::Controller& primary)
{
  if (Buttons::ArmUp.isPressing()) { arm.move_voltage(12000); }
  else if (Buttons::ArmDown.isPressing()) { arm.move_voltage(-12000); }
  else { arm.brake(); }
  if (arm.get_position() > armHeight && !armPneu1.is_extended())
  {
    armPneu1.extend();
    armPneu2.extend();
  }
  else if (arm.get_position() < armHeight && armPneu1.is_extended())
  {
    armPneu1.retract();
    armPneu2.retract();
  }
}
}  // namespace Arm
}  // namespace Systems