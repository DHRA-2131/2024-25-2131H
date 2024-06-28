#pragma once
#include "lib2131/utilities/Pose.hpp"
#include "lib2131/utilities/Units.h"
namespace lib2131::utilities
{
using namespace units::literals;
struct RobotState
{
  Pose Position = {0_in, 0_in, 0_rad};
  Pose Velocity = {0_in, 0_in, 0_rad};
  Pose Acceleration = {0_in, 0_in, 0_rad};
};

}  // namespace lib2131::utilities