#pragma once

#include <cstdlib>
#include <memory>
#include <vector>

#include "lib2131/exit_condition/AbstractExitCondition.hpp"
#include "lib2131/utilities/Angle.hpp"
#include "lib2131/utilities/Motion.hpp"
#include "lib2131/utilities/Pose.hpp"

namespace lib2131::controller
{
class AbstractController
{
 protected:  // variables
  // Exit Conditions
  std::vector<std::shared_ptr<exit_condition::AbstractExitCondition>>
      m_linearExitConditions;
  std::vector<std::shared_ptr<exit_condition::AbstractExitCondition>>
      m_angularExitConditions;

  bool m_canExit;
  // Positional Info
  utilities::Pose m_target;
  utilities::Pose m_error;

  // Stationary Turn Info
  utilities::Angle m_angularTarget;
  utilities::Angle m_angularError;

 public:  // constructors
  AbstractController(std::vector<std::shared_ptr<exit_condition::AbstractExitCondition>>
                         linearExitConditions,
                     std::vector<std::shared_ptr<exit_condition::AbstractExitCondition>>
                         angularExitConditions)
      : m_linearExitConditions(std::move(linearExitConditions)),
        m_angularExitConditions(std::move(angularExitConditions)),
        m_canExit(false)
  {
  }

 public:  // functions
  bool canExit() { return m_canExit; }

  utilities::Pose getError(utilities::Pose currentPose) { return m_target - currentPose; }

  utilities::Angle getAngularError(utilities::Angle currentAngle)
  {
    return m_angularTarget - currentAngle;
  }

  void setTarget(utilities::Pose target, utilities::Pose currentPose = {},
                 bool relative = false)
  {
    if (relative) { target.rotate(currentPose.theta * -1); }
    this->m_target = target;
    for (const auto& ec : this->m_linearExitConditions) { ec->setTarget(target); }
  }

  void setAngularTarget(utilities::Angle target, utilities::Pose currentPose,
                        bool relative = false)
  {
    if (relative) { target += currentPose.theta; }
    this->m_angularTarget = target;
    for (const auto& ec : this->m_angularExitConditions) { ec->setTarget(target); }
  }

 public:  // Overloads
  virtual utilities::Motion getOutput(utilities::Pose currentPose, bool reverse,
                                      bool thru, double deltaTime) = 0;
  virtual utilities::Motion getAngleOutput(utilities::Angle currentAngle, bool reverse,
                                           bool thru, double deltaTime) = 0;
};
}  // namespace lib2131::controller
