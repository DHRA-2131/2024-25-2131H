#pragma once

#include "lib2131/utilities/Pose.hpp"
#include "lib2131/utilities/RobotState.hpp"

namespace lib2131::odometry
{
class AbstractOdometry
{
  using angle_t = units::angle::radian_t;
  using distance_t = units::length::inch_t;

 protected:  // States
  utilities::RobotState m_currentState;
  utilities::RobotState m_deltaState;
  utilities::RobotState m_lastState;

 protected:  // Angle
  angle_t m_currentTheta;
  angle_t m_lastTheta;
  angle_t m_deltaTheta;

 public:  // Constructors
  AbstractOdometry() {}

 public:  // Functions
  void setState(utilities::RobotState newState) { m_currentState = newState; }
  utilities::RobotState getState() { return m_currentState; }

  virtual void update(units::time::millisecond_t deltaTime) = 0;
  virtual void calibrate() = 0;

 protected:  // Functions
  void updateStates(utilities::Pose deltaPosition, angle_t& avgTheta,
                    units::time::millisecond_t deltaTime)
  {
    // Rotate Delta
    deltaPosition.pos.rotate(avgTheta);

    // Update Robot State
    this->m_lastState = this->m_currentState;
    // Calculate Global Change in Position
    this->m_deltaState.Position = deltaPosition;
    // Position
    this->m_currentState.Position += m_deltaState.Position;
    //* DeltaTime is in MilliSeconds
    // Calculate Rate of Change in Position (Velocity)
    this->m_deltaState.Velocity =
        (m_currentState.Position - m_lastState.Position) * deltaTime.value() / 1000.0;
    this->m_currentState.Velocity += m_deltaState.Velocity;

    // Calculate Rate of Change in Velocity (Acceleration)
    this->m_deltaState.Acceleration =
        (m_currentState.Velocity - m_lastState.Velocity) * deltaTime.value() / 1000.0;
    this->m_currentState.Acceleration += m_deltaState.Acceleration;
    // Update Last Data
    this->m_lastTheta = this->m_currentTheta;
  }

  distance_t calculateChordLength(distance_t length, distance_t offset, angle_t theta)
  {
    if (theta == angle_t(0)) { return length; }
    else
    {
      distance_t radius(offset.value() + (length.value() / theta.value()));
      return 2 * radius * units::math::sin(theta / 2.0);
    }
  }
};
}  // namespace lib2131::odometry
