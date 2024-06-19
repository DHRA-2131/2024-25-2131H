#pragma once

#include "lib2131/exit_condition/AbstractExitCondition.hpp"

namespace lib2131::exit_condition
{
class ErrorExitCondition : public AbstractExitCondition
{
 private:  // Variables
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
    if (thru) { return (currentPose.magnitude(m_target) < m_thruTolerance); }
    else { return (currentPose.magnitude(m_target) < m_stopTolerance); }
  }
};
}  // namespace lib2131::exit_condition
