#pragma once

#include <sys/types.h>

#include <cmath>

namespace lib2131::utilities
{
class MotionProfile
{
 private:  // Variables
  // Parameters
  const double m_distance;     // Inches
  const double m_maxVelocity;  // Inches / Seconds
  const double m_maxAccel;     // Inches / Seconds * Seconds
  const double m_maxDecel;     // Inches / Seconds * Seconds

  // Calculated Distances
  const double m_minDistance;    // Inches
  const double m_accelDistance;  // Inches
  const double m_decelDistance;  // Inches
  const double m_coastDistance;  // Inches

  // Calculated Times
  const double m_accelTime;  // Seconds
  const double m_decelTime;  // Seconds
  const double m_coastTime;  // Seconds
  const double m_totalTime;  // Seconds

 public:  // Constructors
          /**
           * @brief Generate Motion Profile off of a distance, maxVelocity, maxAccel, and maxDecel
           * @param distance Distance in inches
           * @param maxVelocity Maximum Velocity in Inches / Second
           * @param maxAccel Maximum Acceleration in Inches / Second * Second
           * @param maxDecel Maximum Deceleration in Inches / Second * Second
           */
  MotionProfile(double distance, double maxVelocity, double maxAccel, double maxDecel)
      : m_distance(distance),
        m_maxVelocity(maxVelocity),
        m_maxAccel(fabs(maxAccel)),
        m_maxDecel(fabs(maxDecel)),
        m_minDistance(maxVelocity * maxVelocity / maxAccel),
        m_accelDistance(maxVelocity * maxVelocity / 2 * maxAccel),
        m_decelDistance(maxVelocity * maxVelocity / 2 * maxDecel),
        m_coastDistance(distance - m_accelDistance - m_decelDistance),
        m_accelTime((maxVelocity / maxAccel)),
        m_decelTime((maxVelocity / maxDecel)),
        m_coastTime((distance / maxVelocity)),
        m_totalTime((m_accelTime + m_coastTime + m_decelTime))
  {
  }

 public:  // Methods
  const double getTotalTime() { return m_totalTime; }
  const double getDistance(uint Msec)
  {
    Msec /= 1000;  // Convert to Seconds
    if (0 < Msec && Msec < m_accelTime) { return 0.5 * m_maxAccel * Msec * Msec; }
    else if (m_accelTime < Msec && Msec < m_accelTime + m_coastTime)
    {
      return m_maxVelocity * (Msec - m_accelTime / 2);
    }
    else if (m_accelTime + m_coastTime < Msec && Msec < m_totalTime)
    {
      return m_maxVelocity * (Msec - m_accelTime / 2) -
             0.5 * m_maxDecel * pow(Msec - m_accelTime - m_coastTime, 2);
    }
    else { return 0.0; }
  }

  const double getVelocity(uint Msec)
  {
    Msec /= 1000;  // Convert to Seconds
    if (0 < Msec && Msec < m_accelTime) { return m_maxAccel * Msec; }
    else if (m_accelTime < Msec && Msec < m_accelTime + m_coastTime)
    {
      return m_maxVelocity;
    }
    else if (m_accelTime + m_coastTime < Msec && Msec < m_totalTime)
    {
      return m_maxVelocity - m_maxDecel * (Msec - m_accelTime - m_coastTime);
    }
    else { return 0.0; }
  }

  const double getAcceleration(uint Msec)
  {
    Msec /= 1000;  // Convert to Seconds
    if (0 < Msec && Msec < m_accelTime) { return m_maxAccel; }
    else if (m_accelTime < Msec && Msec < m_accelTime + m_coastTime) { return 0.0; }
    else if (m_accelTime + m_coastTime < Msec && Msec < m_totalTime)
    {
      return -m_maxDecel;
    }
    else { return 0.0; }
  }
};
}  // namespace lib2131::utilities