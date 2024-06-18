#pragma once
#include "Pose.hpp"
namespace lib2131::utilities
{

struct RobotState
{
  Pose Position = {0, 0, {0, false}};
  Pose Velocity = {0, 0, {0, false}};
  Pose Acceleration = {0, 0, {0, false}};
};

}  // namespace lib2131::utilities
