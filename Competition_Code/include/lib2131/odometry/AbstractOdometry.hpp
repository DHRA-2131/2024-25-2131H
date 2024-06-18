#pragma once
#include "lib2131/utilities/Angle.hpp"
#include "lib2131/utilities/Pose.hpp"
#include "lib2131/utilities/RobotState.hpp"

namespace lib2131::odometry
{
class AbstractOdometry
{
 protected:  // States
  utilities::RobotState m_currentState;
  utilities::RobotState m_deltaState;
  utilities::RobotState m_lastState;

 protected:  // Angle
  utilities::Angle m_currentTheta;
  utilities::Angle m_lastTheta;
  utilities::Angle m_deltaTheta;

 public:  // Constructors
  AbstractOdometry() {}

 public:  // Functions
  void setState(utilities::RobotState newState) { m_currentState = newState; }
  utilities::RobotState getState() { return m_currentState; }

  virtual void update(double deltaTime) = 0;
  virtual void calibrate() = 0;

 protected:  // Functions
  void updateStates(utilities::Pose deltaPosition, double deltaTime)
  {
    // Update Robot State
    this->m_lastState = this->m_currentState;
    // Calculate Global Change in Position
    this->m_deltaState.Position = deltaPosition;
    // Position
    this->m_currentState.Position += m_deltaState.Position;
    //* DeltaTime is in MilliSeconds
    // Calculate Rate of Change in Position (Velocity)
    this->m_deltaState.Velocity =
        (m_currentState.Position - m_lastState.Position) * deltaTime / 1000.0;
    this->m_currentState.Velocity += m_deltaState.Velocity;

    // Calculate Rate of Change in Velocity (Acceleration)
    this->m_deltaState.Acceleration =
        (m_currentState.Velocity - m_lastState.Velocity) * deltaTime / 1000.0;
    this->m_currentState.Acceleration += m_deltaState.Acceleration;
    // Update Last Data
    this->m_lastTheta = this->m_currentTheta;
  }

  double calculateChordLength(double length, double offset, utilities::Angle theta)
  {
    if (theta.getRadians() == 0) { return length; }
    else
    {
      double radius = offset + (length / theta.getRadians());
      return 2 * radius * sin(theta.getRadians() / 2);
    }
  }
};
}  // namespace lib2131::odometry
