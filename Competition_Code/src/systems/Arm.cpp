#include "systems/Arm.hpp"

#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"
#include "pros/abstract_motor.hpp"

namespace Systems
{
namespace Arm
{
bool debug(1);

std::vector<double> positions = {
    0,    // Empty
    170,  // Loaded
    230,  // Loading
    500   // Score
};

size_t index(0);

/**
 * @brief Initialize Arm, Runs code to set up the arm.
 *
 */
void init()
{
  arm.set_brake_mode(pros::MotorBrake::hold);  // Set to hold
  arm.tare_position();                         // Take zero position of motor
  arm.set_encoder_units(pros::MotorEncoderUnits::deg);
}

/**
 * @brief Tele-Operation (Driver / Operator Control)
 *
 */
void teleOp()
{
  if (!debug)
  {
    if (Buttons::ArmUp.changedToPressed())
    {
      index++;
      arm.move_absolute(positions[index], 100);
    }
    else if (Buttons::ArmDown.changedToPressed())
    {
      index--;
      arm.move_absolute(positions[index], 100);
    }
  }
  else
  {
    if (Buttons::ArmUp.isPressing()) { arm.move_voltage(12000); }
    else if (Buttons::ArmDown.isPressing()) { arm.move_voltage(-12000); }
    else { arm.brake(); }
  }
}
}  // namespace Arm
}  // namespace Systems