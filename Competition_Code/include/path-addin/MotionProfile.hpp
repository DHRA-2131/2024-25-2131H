#pragma once

#include <cmath>
#include <limits>
namespace Pathing
{
class MotionProfile
{
  // Distance is in in
  // Velocity is in in/s
  // Acceleration is in in/s^2
  // Time is in seconds

 private:  // Variables
  // Parameters
  double m_distance;
  double m_maxVelocity;
  double m_initialVelocity;
  double m_endVelocity;
  double m_maxAccel;
  double m_maxDecel;

  // Calculated Distances
  double m_accelDistance;
  double m_decelDistance;
  double m_coastDistance;

  // Calculated Times
  double m_accelTime;
  double m_decelTime;
  double m_coastTime;
  double m_totalTime;
  double m_interceptTime = std::numeric_limits<double>::infinity();

 public:  // Constructors
  MotionProfile(double distance, double maxVelocity, double maxAccel, double maxDecel, double initalVelocity = 0, double endVelocity = 0)
      : m_distance(distance),
        m_maxVelocity(maxVelocity),
        m_maxAccel(std::fabs(maxAccel)),
        m_maxDecel(std::fabs(maxDecel)),
        m_initialVelocity(initalVelocity),
        m_endVelocity(endVelocity),
        m_accelTime((m_maxVelocity - m_initialVelocity) / m_maxAccel),
        m_decelTime((m_maxVelocity - m_endVelocity) / m_maxDecel)
  {
    m_accelDistance = (0.5 * m_maxAccel * std::pow(m_accelTime, 2) + m_initialVelocity * m_accelTime);
    m_decelDistance = (m_maxVelocity * m_decelTime - 0.5 * m_maxDecel * std::pow(m_decelTime, 2));
    m_coastDistance = (m_distance - m_accelDistance - m_decelDistance);
    m_coastTime = (m_coastDistance / m_maxVelocity);
    m_totalTime = (m_accelTime + m_coastTime + m_decelTime);

    if (m_coastTime < 0)
    {
      m_interceptTime = (m_maxVelocity + m_maxDecel * m_accelTime + m_maxDecel * m_coastTime) / (m_maxAccel + m_maxDecel);
      m_maxVelocity = m_maxAccel * m_interceptTime;
      m_accelDistance = 0.5 * m_maxAccel * m_accelTime * m_accelTime;
      m_decelDistance = m_maxVelocity * m_decelTime - 0.5 * m_maxDecel * m_decelTime * m_decelTime;
      m_coastDistance = 0.0;
      m_accelTime = m_interceptTime;
      m_decelTime = m_totalTime - m_interceptTime;
      m_coastTime = 0.0;
      m_totalTime = m_accelTime + m_decelTime;
    }
  }

 public:  // Methods
  double getTotalTime() { return m_totalTime; }
  bool isTrapezoid() { return std::isinf(m_interceptTime); }

  double getDistance(double time)
  {
    time /= 1000.0;
    if (0.0 <= time && time < m_accelTime) { return 0.5 * m_maxAccel * std::pow(time, 2) + m_initialVelocity * time; }
    else if (m_accelTime <= time && time < m_accelTime + m_coastTime) { return m_maxVelocity * (time - m_accelTime) + m_decelDistance; }
    else if (m_accelTime + m_coastTime <= time && time < m_totalTime)
    {
      return m_maxVelocity * (time - m_accelTime) - 0.5 * m_maxDecel * std::pow(time, 2 - m_accelTime - m_coastTime) + m_decelDistance;
    }
    else { return m_accelDistance + m_decelDistance + m_coastDistance; }
  }

  double getVelocity(double time)
  {
    time /= 1000.0;
    if (0.0 <= time && time < m_accelTime) { return m_maxAccel * time + m_initialVelocity; }
    else if (m_accelTime <= time && time < m_accelTime + m_coastTime) { return m_maxVelocity; }
    else if (m_accelTime + m_coastTime <= time && time < m_totalTime)
    {
      return m_maxVelocity - m_maxDecel * (time - m_accelTime - m_coastTime);
    }
    else { return 0.0; }
  }

  double getAcceleration(double time)
  {
    time /= 1000.0;
    if (0.0 <= time && time < m_accelTime) { return m_maxAccel; }
    else if (m_accelTime <= time && time < m_accelTime + m_coastTime) { return 0.0; }
    else if (m_accelTime + m_coastTime <= time && time < m_totalTime) { return -m_maxDecel; }
    else { return 0.0; }
  }
};
}  // namespace Pathing