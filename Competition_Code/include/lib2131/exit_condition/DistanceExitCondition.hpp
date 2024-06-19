#pragma once

#include "lib2131/exit_condition/AbstractExitCondition.hpp"
#include "lib2131/utilities/Pose.hpp"

namespace lib2131::exit_condition
{
class ErrorExitCondition : public AbstractExitCondition
{
 private:  // Variables
  utilities::Pose m_target;
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
    double distance = currentPose.magnitude(m_target);
    // X is Forward Error
    if (thru) { return (distance < m_thruTolerance); }
    else { return (distance < m_stopTolerance); }
  }
};
}  // namespace lib2131::exit_condition
