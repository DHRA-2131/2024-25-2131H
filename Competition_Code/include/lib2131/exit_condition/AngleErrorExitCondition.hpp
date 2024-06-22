#pragma once

#include "lib2131/exit_condition/AbstractExitCondition.hpp"
#include "lib2131/utilities/Angle.hpp"

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
  void setTarget(utilities::Angle target) override { this->m_target = target; }
  bool canExit(utilities::Angle currentAngle, bool thru) override
  {
    return thru ? ((this->m_target - currentAngle).getDegrees() < m_thruTolerance)
                : ((this->m_target - currentAngle).getDegrees() < m_stopTolerance);
  }
};
}  // namespace lib2131::exit_condition
