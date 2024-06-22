#pragma once

#include "lib2131/utilities/Pose.hpp"

namespace lib2131::exit_condition
{
class AbstractExitCondition
{
 public:  // Constructors
  AbstractExitCondition() {}

 public:  // Checks / Functions
  virtual void setTarget(utilities::Pose target) = 0;
  virtual void setTarget(utilities::Angle target) = 0;
  virtual bool canExit(utilities::Pose currentPose, bool thru) = 0;
  virtual bool canExit(utilities::Angle currentAngle, bool thru) = 0;
};
}  // namespace lib2131::exit_condition