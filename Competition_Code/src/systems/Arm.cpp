#include "systems/Arm.hpp"

#include <algorithm>
#include <cstddef>

#include "lemlib/pid.hpp"
#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"
#include "pros/abstract_motor.hpp"

namespace Systems
{
namespace Arm
{
bool debug(0);

/**
 * @brief Positions of Arm at different stages
 *
 *
 */
std::vector<double> positions = {
    0,   // Empty
    14,  // Loading
    100  // Score
};

size_t index(0);

lemlib::PID armPID(15, 0.001, 1);

/**
 * @brief Initialize Arm, Runs code to set up the arm.
 *
 */
void init()
{
  arm.set_brake_mode(pros::MotorBrake::hold);    // Set to hold
  arm.set_encoder_units(pros::MotorUnits::deg);  // Set Motor Units -> Degrees
  arm.move_relative(-10, 100);                   // Force Motor into stop
  pros::delay(500);                              // Allow Motor Time to get forced into stop
  arm.tare_position();                           // Take zero position of motor
}

/**
 * @brief Tele-Operation (Driver / Operator Control)
 *
 */
void teleOp()
{
  if (!debug)  // Competition Code
  {
    if (Buttons::ArmUp.changedToPressed())  // If Arm UP
    {
      index++;                                                // Increase Index by 1
      index = std::min(index, size_t(positions.size() - 1));  // Keep Index in bounds
      arm.move_absolute(positions[index], 100);               // Move Arm
    }
    else if (Buttons::ArmDown.changedToPressed())
    {
      index--;                                   // Decrease Index by 1
      index = std::max(index, size_t(0));        // Keep index in bounds
      arm.move_absolute(positions[index], 100);  // Move Arm
    }
  }
  else  // Debug Code
  {
    // If Arm Up, Then spin arm at max voltage
    if (Buttons::ArmUp.isPressing()) { arm.move_voltage(12000); }
    // If Arm Down, Then spin arm in reverse at max voltage
    else if (Buttons::ArmDown.isPressing()) { arm.move_voltage(-12000); }
    // If neither button is pressed then stop (Stop mode is on HOLD)
    else { arm.brake(); }
  }

  if (Buttons::Doinkler.changedToPressed()) { doinkler.toggle(); }
}

pros::Task armThread(
    []() {
      while (true)
      {
        double out = armPID.update(positions[index] - arm.get_position() / 4.0);
        arm.move_voltage(out * 100);
        std::cout << arm.get_position() / 4.0 << std::endl;
        pros::delay(10);
      }
    },
    "ARM THREAD");

}  // namespace Arm
}  // namespace Systems