#pragma once

#include "lib2131/exit_condition/AbstractExitCondition.hpp"
#include "utilities/Angle.hpp"

namespace lib2131::exit_condition
{
class ErrorExitCondition : public AbstractExitCondition
{
 private:  // Variables
  utilities::Angle m_target;
  double m_stopTolerance;
  double m_thruTolerance;

 public:  // Constructors
  ErrorExitCondition(double stopTolerance, double thruTolerance)
      : AbstractExitCondition(),
        m_stopTolerance(stopTolerance),
        m_thruTolerance(thruTolerance)
  {
  }

 public:  // Functions
  bool canExit(utilities::Pose currentPose, bool thru) override
  {
    if (thru)
    {
      return ((this->m_target - currentPose.theta).getDegrees() < m_thruTolerance);
    }
    else { return ((this->m_target - currentPose.theta).getDegrees() < m_stopTolerance); }
  }
};
}  // namespace lib2131::exit_condition
