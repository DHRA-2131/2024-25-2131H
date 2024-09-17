#include "systems/Clamp.hpp"

#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"
#include "pros/misc.h"

namespace Systems
{
namespace Clamp
{
/**
 * @brief Tele-Operation (also known as Driver / Operator)
 *
 */
void teleOp()
{
  // If a button's state is changed from released to pressed then toggle clamp
  // Button state is determined using the Buttons::ButtonDetector
  if (Buttons::ClampToggle.changedToPressed()) { clamp.toggle(); }
}
}  // namespace Clamp
}  // namespace Systems