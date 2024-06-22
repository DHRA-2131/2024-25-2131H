#pragma once

#include <algorithm>
#include <vector>

#include "lib2131/controller/AbstractController.hpp"
#include "lib2131/exit_condition/AbstractExitCondition.hpp"
#include "lib2131/utilities/Angle.hpp"
#include "lib2131/utilities/PID.hpp"
#include "lib2131/utilities/Pose.hpp"

namespace lib2131::controller
{
class PIDController : public AbstractController
{
 private:  // Variables
  PID m_linearPID, m_angularPID;
  double m_angleLockDistance;

 public:  // Constructors
  PIDController(std::vector<std::shared_ptr<exit_condition::AbstractExitCondition>>
                    linearExitConditions,
                std::vector<std::shared_ptr<exit_condition::AbstractExitCondition>>
                    angularExitConditions,
                PID linearPID, PID angularPID, double angleLockDist)
      : AbstractController(linearExitConditions, angularExitConditions),
        m_linearPID(linearPID),
        m_angularPID(angularPID),
        m_angleLockDistance(angleLockDist)
  {
  }

 public:  // Functions
  utilities::Motion getLinearOutput(utilities::Pose currentPose, bool reverse, bool thru,
                                    double deltaTime) override
  {
    double linearOut, angularOut;

    // Convert to useful Direction
    int direction = reverse ? -1 : 1;

    // Calculate Error
    this->m_error = m_target - currentPose;
    double linearError = this->m_target.magnitude(currentPose);

    // Initialize Angle Error
    utilities::Angle angleError(
        std::atan2(m_error.y, m_error.y) - currentPose.theta.getRadians(), false);

    // Flip Heading and recalculate angle Error
    if (reverse)
    {
      angleError = {std::atan2(-m_error.y, -m_error.y) - currentPose.theta.getRadians(),
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

    // All Exit Conditions say exit is possible
    if (std::all_of(this->m_linearExitConditions.begin(),
                    this->m_linearExitConditions.end(),
                    [&](std::shared_ptr<exit_condition::AbstractExitCondition> ec) {
                      return ec->canExit(currentPose, thru);
                    }))
    {
      // Then Allow Exit
      this->m_canExit = true;
    }

    // Return Output
    return {linearError, angularOut};
  }

  utilities::Motion getAngleOutput(utilities::Angle currentAngle, bool reverse, bool thru,
                                   double deltaTime) override
  {
    // Use Rear Heading?
    if (reverse) { currentAngle += utilities::Angle(180, true); }

    // Calculate Angle Error
    utilities::Angle angleError = this->getAngularError(currentAngle);

    // Force Shortest Direction
    if (fabs(angleError.getDegrees()) > 180)
    {
      angleError.getDegrees() < 0 ? angleError += utilities::Angle(360, true)
                                  : angleError -= utilities::Angle(360, false);
    }

    double angularOutput = this->m_angularPID.calc(angleError.getRadians(), deltaTime);

    if (std::all_of(this->m_angularExitConditions.begin(),
                    this->m_angularExitConditions.end(),
                    [&](std::shared_ptr<exit_condition::AbstractExitCondition> ec) {
                      return ec->canExit(currentAngle, thru);
                    }))
    {
      // Then Allow Exit
      this->m_canExit = true;
    }

    return {0, angularOutput};
  }
};
}  // namespace lib2131::controller