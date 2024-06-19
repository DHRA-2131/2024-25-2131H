#pragma once

#include <memory>

#include "lib2131/exit_condition/AbstractExitCondition.hpp"
#include "lib2131/odometry/AbstractOdometry.hpp"
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
  std::shared_ptr<odometry::AbstractOdometry> m_pOdometry;
  utilities::Pose m_target;
  utilities::Pose m_error;

  // Stationary Turn Info
  utilities::Angle m_angularTarget;
  utilities::Angle m_angularError;

 public:  // constructors
  AbstractController(std::shared_ptr<odometry::AbstractOdometry> odometry,
                     std::shared_ptr<exit_condition::AbstractExitCondition> linearExit,
                     std::shared_ptr<exit_condition::AbstractExitCondition> angularExit)
      : m_pOdometry(std::move(odometry)),
        m_pLinearExit(std::move(linearExit)),
        m_pAngularExit(angularExit)
  {
  }

 public:  // functions
  utilities::Pose getError() { return m_target - m_pOdometry->getState().Position; }

  utilities::Angle getAngularError()
  {
    return m_angularTarget - m_pOdometry->getState().Position.theta;
  }

  void setTarget(utilities::Pose target, bool relative)
  {
    if (relative) { target.rotate(m_pOdometry->getState().Position.theta * -1); }
    this->m_target = target;
  }

  void setAngularTarget(utilities::Angle target, bool relative)
  {
    if (relative) { target += m_pOdometry->getState().Position.theta; }
    this->m_angularTarget = target;
  }

 public:  // Overloads
  virtual utilities::Motion getOutput(bool reverse, bool thru, double deltaTime) = 0;
};
}  // namespace lib2131::controller
