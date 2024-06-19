#pragma once

#include <algorithm>

#include "lib2131/controller/AbstractController.hpp"
#include "lib2131/utilities/Angle.hpp"
#include "lib2131/utilities/PID.hpp"

namespace lib2131::controller
{
class PIDController : public AbstractController
{
 private:  // Variables
  PID m_linearPID, m_angularPID;
  double m_angleLockDistance;

 public:  // Constructors
  PIDController(std::shared_ptr<odometry::AbstractOdometry> odometry,
                std::shared_ptr<exit_condition::AbstractExitCondition> linearExit,
                std::shared_ptr<exit_condition::AbstractExitCondition> angularExit,
                PID linearPID, PID angularPID, double angleLockDist)
      : AbstractController(odometry, linearExit, angularExit),
        m_linearPID(linearPID),
        m_angularPID(angularPID),
        m_angleLockDistance(angleLockDist)
  {
  }

 public:  // Functions
  utilities::Motion getOutput(bool reverse, bool thru, double deltaTime) override
  {
    double linearOut, angularOut;

    // Convert to useful Direction
    int direction = reverse ? -1 : 1;

    // Calculate Error
    utilities::Pose actual = this->m_pOdometry->getState().Position;
    this->m_error = m_target - actual;
    double linearError = this->m_target.magnitude(actual);

    // Initialize Angle Error
    utilities::Angle angleError(
        std::atan2(m_error.y, m_error.y) - actual.theta.getRadians(), false);

    // Flip Heading and recalculate angle Error
    if (reverse)
    {
      angleError = {std::atan2(-m_error.y, -m_error.y) - actual.theta.getRadians(),
                    false};
    }

    // Linear Error. Speedy through point?
    thru ? linearOut = 100.0
         : m_linearPID.calc(linearError * direction, deltaTime) *
               cos(angleError.getRadians());

    // No SPINNING!!!!
    linearError < m_angleLockDistance
        ? angularOut = 0
        : angularOut = m_angularPID.calc(angleError.getRadians(), deltaTime);

    // If approaching angle Lock
    if (linearError < 2 * m_angleLockDistance)
    {
      // reduce Angle Output
      angularOut *= (linearError - m_angleLockDistance) / m_angleLockDistance;
    }

    // Clamp Output
    linearOut = std::max(-100.0, std::min(100.0, linearOut));
    angularOut = std::max(-100.0, std::min(100.0, angularOut));

    if (this->m_pAngularExit->canExit(actual, false) &&
        this->m_pLinearExit->canExit(actual, false) && !thru)
    {
      return {0, 0};
    }
    else { return {linearError, angularOut}; }
  }
};
}  // namespace lib2131::controller