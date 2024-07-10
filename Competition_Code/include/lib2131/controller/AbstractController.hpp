#pragma once

#include "lib2131/chassis/ChassisParameters.hpp"
#include "lib2131/controller/ExitCondition.hpp"
#include "lib2131/utilities/MotionOutputs.hpp"
#include "lib2131/utilities/Pose.hpp"

namespace lib2131::controller
{
class AbstractController
{
 protected:  // variables
  // Positional Info
  utilities::Pose m_target;

  // Stationary Turn Info
  units::angle::radian_t m_angularTarget;

 public:  // Variables
  // Exit Conditions
  exit_condition::ExitCondition exitCondition;

 public:  // constructors
  AbstractController(const exit_condition::ExitCondition& exitCondition)
      : exitCondition(exitCondition)
  {
  }

 public:  // functions
  void setTarget(utilities::Pose newTarget)
  {
    m_target = newTarget;
    this->exitCondition.setTarget(newTarget);
  }

 public:  // Overloads
  virtual utilities::WheelVelocities getOutput(utilities::Pose currentPose, bool reverse,
                                               bool thru, bool checkAngle,
                                               double deltaTime) = 0;

  virtual void setConstraints(lib2131::chassis::ChassisParameters Constraints) = 0;
};
}  // namespace lib2131::controller