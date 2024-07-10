#pragma once

#include "lib2131/utilities/Units.h"

namespace lib2131::utilities
{
using linear_velocity_t = units::velocity::feet_per_second;
using angular_velocity_t = units::angular_velocity::radians_per_second_t;

struct WheelVelocities
{
  linear_velocity_t leftWheelVelocity;
  linear_velocity_t rightWheelVelocity;
};

}  // namespace lib2131::utilities