#pragma once

namespace Systems
{
namespace Arm
{
#define GEAR_RATIO 3
void init();
void teleOp();
void setPosition(int newIndex);
}  // namespace Arm
}  // namespace Systems