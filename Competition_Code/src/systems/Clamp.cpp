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
  autoClampEnabled = false;
  // If a button's state is changed from released to pressed then toggle clamp
  // Button state is determined using the Buttons::ButtonDetector
  if (Buttons::ClampToggle.changedToPressed()) { Clamp::pneumatic.toggle(); }
}

pros::Task armThread(
    []() {
      while (true)
      {
        if ((autoClampEnabled && goalDetector.get_distance() < 30) || Clamp::pneumatic.is_extended()) { pneumatic.extend(); }
        else if (autoClampEnabled) { pneumatic.retract(); }
        pros::delay(10);
      }
    },
    "ARM THREAD");
void enableAutoClamp() { autoClampEnabled = true; }
void disableAutoClamp()
{
  autoClampEnabled = false;
  Clamp::pneumatic.retract();
}
}  // namespace Clamp
}  // namespace Systems