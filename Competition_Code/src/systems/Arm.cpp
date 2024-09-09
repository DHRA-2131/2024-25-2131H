#include "systems/Arm.hpp"

#include "main/RobotConfig.hpp"
#include "pros/abstract_motor.hpp"

namespace Systems
{
namespace Arm
{
/**
 * @brief Initialize Arm, Runs code to set up the arm.
 *
 */
void init()
{
  arm.set_brake_mode(pros::MotorBrake::hold);  // Set to hold
  arm.tare_position();                         // Take zero position of motor
}

/**
 * @brief Tele-Operation (Driver / Operator Control)
 *
 */
void teleOp() {}
}  // namespace Arm
}  // namespace Systems