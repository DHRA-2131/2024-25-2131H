#pragma once

#include "pros/misc.hpp"

namespace Systems
{
namespace Arm
{
void init();
void teleOp(pros::Controller& primary);
}  // namespace Arm
}  // namespace Systems