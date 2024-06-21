#pragma once

#include <cmath>

namespace lib2131::utilities
{
class MotionProfile
{
 private:  // Variables
  // Parameters
  const double m_distance;     // Inches
  const double m_maxVelocity;  // Inches / Msec
  const double m_maxAccel;     // Inches / Msec * Msec
  const double m_maxDecel;     // Inches / Msec * Msec

  // Calculated Distances
  const double m_minDistance;    // Inches
  const double m_accelDistance;  // Inches
  const double m_decelDistance;  // Inches
  const double m_coastDistance;  // Inches

  // Calculated Times
  const double m_accelTime;  // Msec
  const double m_decelTime;  // Msec
  const double m_coastTime;  // Msec
  const double m_totalTime;  // Msec

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
        m_maxVelocity(maxVelocity * 1000),
        m_maxAccel(fabs(maxAccel * 1000 * 1000)),
        m_maxDecel(fabs(maxDecel * 1000 * 1000)),
        m_minDistance(maxVelocity * maxVelocity / maxAccel),
        m_accelDistance(maxVelocity * maxVelocity / 2 * maxAccel),
        m_decelDistance(maxVelocity * maxVelocity / 2 * maxDecel),
        m_coastDistance(distance - m_accelDistance - m_decelDistance),
        m_accelTime((maxVelocity / maxAccel) * 1000),
        m_decelTime((maxVelocity / maxDecel) * 1000),
        m_coastTime((distance / maxVelocity) * 1000),
        m_totalTime((m_accelTime + m_coastTime + m_decelTime) * 1000)
  {
  }

 public:  // Methods
  const double getDistance(double Msec)
  {
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

  const double getVelocity(double Msec)
  {
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

  const double getAcceleration(double Msec)
  {
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