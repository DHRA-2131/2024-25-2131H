#pragma once

#include <memory>

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
  std::shared_ptr<exit_condition::AbstractExitCondition> m_pLinearExit;
  std::shared_ptr<exit_condition::AbstractExitCondition> m_pAngularExit;

  // Positional Info
  utilities::Pose m_target;
  utilities::Pose m_error;

  // Stationary Turn Info
  utilities::Angle m_angularTarget;
  utilities::Angle m_angularError;

 public:  // constructors
  AbstractController(std::shared_ptr<exit_condition::AbstractExitCondition> linearExit,
                     std::shared_ptr<exit_condition::AbstractExitCondition> angularExit)
      : m_pLinearExit(std::move(linearExit)), m_pAngularExit(angularExit)
  {
  }

 public:  // functions
  utilities::Pose getError(utilities::Pose currentPose) { return m_target - currentPose; }

  utilities::Angle getAngularError(utilities::Pose currentPose)
  {
    return m_angularTarget - currentPose.theta;
  }

  void setTarget(utilities::Pose target, utilities::Pose currentPose = {},
                 bool relative = false)
  {
    if (relative) { target.rotate(currentPose.theta * -1); }
    this->m_target = target;
  }

  void setAngularTarget(utilities::Angle target, utilities::Pose currentPose,
                        bool relative = false)
  {
    if (relative) { target += currentPose.theta; }
    this->m_angularTarget = target;
  }

 public:  // Overloads
  virtual utilities::Motion getOutput(utilities::Pose currentPose, bool reverse,
                                      bool thru, double deltaTime) = 0;
};
}  // namespace lib2131::controller
