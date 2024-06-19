#pragma once

#include "lib2131/utilities/Pose.hpp"

namespace lib2131::exit_condition
{
class AbstractExitCondition
{
 protected:  // Variables
  utilities::Pose m_target;

 public:  // Constructors
  AbstractExitCondition() {}

 public:  // Checks / Functions
  void setTarget(utilities::Pose Target) { m_target = Target; }
  virtual bool canExit(utilities::Pose currentPose, bool thru) = 0;
};
}  // namespace lib2131::exit_condition