#include "systems/Arm.hpp"

#include <algorithm>
#include <cstddef>

#include "lemlib/pid.hpp"
#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"
namespace Systems
{
namespace Arm
{

/**
 * @brief Positions of Arm at different stages
 *
 *
 */
std::vector<double> positions = {
    0,    // Empty
    25,   // Loading 1
    160,  // Wallstake
    210,  // Down / alliance stake
    275   // lowest un tip

};

size_t index(0);

lemlib::PID armPID(5, 0, 15.5, 3, true);

/**
 * @brief Initialize Arm, Runs code to set up the motor.
 *
 */
void init()
{
  motor.set_brake_mode(pros::MotorBrake::hold);    // Set to hold
  motor.set_encoder_units(pros::MotorUnits::deg);  // Set Motor Units -> Degrees
}

/**
 * @brief Tele-Operation (Driver / Operator Control)
 *
 */
void teleOp()
{
  if (Buttons::ArmUp.changedToPressed())  // If Arm UP
  {
    index++;                                                // Increase Index by 1
    index = std::min(index, size_t(positions.size() - 1));  // Keep Index in bounds
    motor.move_absolute(positions[index], 100);             // Move Arm
  }
  else if (Buttons::ArmDown.changedToPressed())
  {
    index--;                                     // Decrease Index by 1
    index = std::max(index, size_t(0));          // Keep index in bounds
    motor.move_absolute(positions[index], 100);  // Move Arm
  }

  if (Buttons::Doinkler.changedToPressed()) { doinkler.toggle(); }

  if (Buttons::Rush.changedToPressed())
  {
    rush1.toggle();
    rush2.toggle();
  }
}

/**
 * @brief Set the target Position of the arm
 *
 * @param newIndex
 */
void setPosition(int newIndex) { index = newIndex; }

pros::Task armThread(
    []() {
      while (true)
      {
        // Get output from PID (Target - Actual (Accounts for gear ratio))
        double out = armPID.update(positions[index] - motor.get_position() / GEAR_RATIO);
        motor.move_voltage(out * 100);  // Output to motor
        pros::delay(10);                // Don't take up CPU resources
      }
    },
    "ARM THREAD");

}  // namespace Arm
}  // namespace Systems