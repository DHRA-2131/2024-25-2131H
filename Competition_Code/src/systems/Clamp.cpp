#include "systems/Clamp.hpp"

#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"

namespace Systems
{
namespace Clamp
{
bool autoClampEnabled(1);  // Is Auto Clamp Enabled?

/**
 * @brief Tele-Operation (also known as Driver / Operator)
 *
 */
void teleOp()
{
  // Disable auto clamp for driver control (Drivers request)
  autoClampEnabled = false;
  // If a button's state is changed from released to pressed then toggle clamp
  // Button state is determined using the Buttons::ButtonDetector
  if (Buttons::ClampToggle.changedToPressed()) { Clamp::pneumatic.toggle(); }
}

pros::Task autoClampThread(
    []() {
      while (true)
      {
        // IF enabled, and goal is detected, then clamp
        if ((autoClampEnabled && goalDetector.get_distance() < 35) || Clamp::pneumatic.is_extended()) { pneumatic.extend(); }
        // Otherwise, retract
        else if (autoClampEnabled) { pneumatic.retract(); }
        pros::delay(10);  // Don't take up CPU resources
      }
    },
    "AUTO CLAMP THREAD");

/**
 * @brief Enable the auto clamp
 *
 */
void enableAutoClamp() { autoClampEnabled = true; }

/**
 * @brief Disable the auto clamp
 *
 */
void disableAutoClamp()
{
  autoClampEnabled = false;
  Clamp::pneumatic.retract();
}
}  // namespace Clamp
}  // namespace Systems