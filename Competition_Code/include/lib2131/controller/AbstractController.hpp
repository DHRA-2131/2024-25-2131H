#pragma once

#include <memory>

#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/utilities/Angle.hpp"
#include "utilities/Pose.hpp"

namespace lib2131::controller
{
class AbstractController
{
 private:  // variables'
  // Positional Info
  std::shared_ptr<odometry::AbstractOdometry> m_pOdometry;
  utilities::Pose m_target;

  // Stationary Turn Info
  utilities::Angle m_angularTarget;

 public:  // constructors
  AbstractController(std::shared_ptr<odometry::AbstractOdometry> Odometry)
      : m_pOdometry(std::move(Odometry))
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

 public:  // overloads
};
}  // namespace lib2131::controller
