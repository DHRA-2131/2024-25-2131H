#include "systems/Clamp.hpp"

#include "main/ButtonConfig.hpp"
#include "main/RobotConfig.hpp"

namespace Systems
{
namespace Clamp
{

void teleOp()
{
  if (Buttons::ClampToggle.changedToPressed()) { clamp.toggle(); }
}
}  // namespace Clamp
}  // namespace Systems