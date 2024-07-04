#pragma once

#include <sys/types.h>

#include <cmath>
#include <limits>

#include "lib2131/utilities/Units.h"

namespace lib2131::utilities
{
using namespace units::literals;

template <class UnitType>
class MotionProfile
{
  using distance_t = units::unit_t<UnitType>;
  using velocity = units::compound_unit<UnitType, units::inverse<units::time::seconds>>;
  using velocity_t = units::unit_t<velocity>;
  using acceleration =
      units::compound_unit<velocity, units::inverse<units::time::seconds>>;
  using acceleration_t = units::unit_t<acceleration>;

  using time_t = units::time::second_t;

 private:  // Variables
  // Parameters
  distance_t m_distance;
  velocity_t m_maxVelocity;
  acceleration_t m_maxAccel;
  acceleration_t m_maxDecel;

  // Calculated Distances
  distance_t m_minDistance;
  distance_t m_accelDistance;
  distance_t m_decelDistance;
  distance_t m_coastDistance;

  // Calculated Times
  time_t m_accelTime;
  time_t m_decelTime;
  time_t m_coastTime;
  time_t m_totalTime;
  time_t m_interceptTime = time_t(std::numeric_limits<double>::infinity());

 public:  // Constructors
  MotionProfile(double distance, double maxVelocity, double maxAccel, double maxDecel)
      : m_distance(distance),
        m_maxVelocity(maxVelocity),
        m_maxAccel(fabs(maxAccel)),
        m_maxDecel(fabs(maxDecel)),
        m_minDistance(maxVelocity * maxVelocity / maxAccel),
        m_accelDistance(maxVelocity * maxVelocity / (2 * fabs(maxAccel))),
        m_decelDistance(maxVelocity * maxVelocity / (2 * fabs(maxDecel))),
        m_coastDistance(m_distance - m_accelDistance - m_decelDistance),
        m_accelTime(maxVelocity / maxAccel),
        m_decelTime(maxVelocity / maxDecel),
        m_coastTime(m_coastDistance / m_maxVelocity),
        m_totalTime(m_accelTime + m_coastTime + m_decelTime)
  {
    if (m_coastTime < 0_s)
    {
      m_interceptTime = m_accelTime + m_coastTime / 2;
      m_maxVelocity = m_maxAccel * m_interceptTime;
      m_accelDistance =
          m_maxVelocity * m_maxVelocity / (2 * units::math::fabs(m_maxAccel));
      m_decelDistance =
          m_maxVelocity * m_maxVelocity / (2 * units::math::fabs(m_maxDecel));
      m_coastDistance = distance_t(0.0);
      m_accelTime = m_interceptTime;
      m_decelTime = m_totalTime - m_interceptTime;
      m_coastTime = time_t(0.0);
      m_totalTime = m_accelTime + m_coastTime + m_decelTime;
    }
  }

 public:  // Methods
  time_t getTotalTime() { return m_totalTime; }
  bool isTrapezoid() { return std::isinf(m_interceptTime.value()); }

  distance_t getDistance(time_t time)
  {
    if (time_t(0) < time && time < m_accelTime)
    {
      return 0.5 * m_maxAccel * units::math::pow<2>(time);
    }
    else if (m_accelTime < time && time < m_accelTime + m_coastTime)
    {
      return m_maxVelocity * (time - m_accelTime / 2);
    }
    else if (m_accelTime + m_coastTime < time && time < m_totalTime)
    {
      return m_maxVelocity * (time - m_accelTime / 2) -
             0.5 * m_maxDecel * units::math::pow<2>(time - m_accelTime - m_coastTime);
    }
    else { return m_accelDistance + m_decelDistance + m_coastDistance; }
  }

  velocity_t getVelocity(time_t time)
  {
    if (time_t(0) < time && time < m_accelTime) { return m_maxAccel * time; }
    else if (m_accelTime < time && time < m_accelTime + m_coastTime)
    {
      return m_maxVelocity;
    }
    else if (m_accelTime + m_coastTime < time && time < m_totalTime)
    {
      return m_maxVelocity - m_maxDecel * (time - m_accelTime - m_coastTime);
    }
    else { return velocity_t(0.0); }
  }

  acceleration_t getAcceleration(time_t time)
  {
    if (time_t(0) < time && time < m_accelTime) { return m_maxAccel; }
    else if (m_accelTime < time && time < m_accelTime + m_coastTime)
    {
      return acceleration_t(0.0);
    }
    else if (m_accelTime + m_coastTime < time && time < m_totalTime)
    {
      return -m_maxDecel;
    }
    else { return acceleration_t(0.0); }
  }
};
}  // namespace lib2131::utilities