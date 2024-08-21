#pragma once

#include "AddedUnits.hpp"
#include "Units.h"

namespace lib2131::utilities
{
struct Velocity
{
  units::velocity::inch_per_second_t linear;
  units::angular_velocity::radians_per_second_t angular;
};
}  // namespace lib2131::utilities