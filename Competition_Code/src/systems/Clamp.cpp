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
  // If a button's state is changed from released to pressed then toggle clamp
  // Button state is determined using the Buttons::ButtonDetector
  if (Buttons::ClampToggle.changedToPressed()) { autoClampEnabled = !autoClampEnabled; }
}

pros::Task armThread(
    []() {
      while (true)
      {
        if (autoClampEnabled && goalDetector.get_distance() < 30) { pneumatic.set_value(1); }
        else { pneumatic.set_value(0); }

        pros::delay(10);
      }
    },
    "ARM THREAD");

}  // namespace Clamp
}  // namespace Systems