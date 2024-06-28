#pragma once

#include <sys/types.h>

#include <cmath>

#include "lib2131/utilities/Units.h"

namespace lib2131::utilities
{
using namespace units::literals;

template <class Distance>
class MotionProfile
{
  using distance_t = units::unit_t<Distance>;
  using velocity = units::compound_unit<Distance, units::inverse<units::time::seconds>>;
  using velocity_t = units::unit_t<velocity>;
  using acceleration =
      units::compound_unit<velocity_t, units::inverse<units::time::seconds>>;
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

 public:  // Constructors
  MotionProfile(distance_t distance, velocity_t maxVelocity, acceleration_t maxAccel,
                acceleration_t maxDecel)
      : m_distance(distance),
        m_maxVelocity(maxVelocity),
        m_maxAccel(fabs(maxAccel)),
        m_maxDecel(fabs(maxDecel)),
        m_minDistance(maxVelocity * maxVelocity / maxAccel),
        m_accelDistance(maxVelocity * maxVelocity / 2 * maxAccel),
        m_decelDistance(maxVelocity * maxVelocity / 2 * maxDecel),
        m_coastDistance(distance - m_accelDistance - m_decelDistance),
        m_accelTime(maxVelocity / maxAccel),
        m_decelTime(maxVelocity / maxDecel),
        m_coastTime(distance / maxVelocity),
        m_totalTime(m_accelTime + m_coastTime + m_decelTime)
  {
  }

 public:  // Methods
  time_t getTotalTime() { return m_totalTime; }
  distance_t getDistance(time_t time)
  {
    if (0_s < time && time < m_accelTime) { return 0.5 * m_maxAccel * time * time; }
    else if (m_accelTime < time && time < m_accelTime + m_coastTime)
    {
      return m_maxVelocity * (time - m_accelTime / 2);
    }
    else if (m_accelTime + m_coastTime < time && time < m_totalTime)
    {
      return m_maxVelocity * (time - m_accelTime / 2) -
             0.5 * m_maxDecel * units::math::pow<2>(time - m_accelTime - m_coastTime);
    }
    else { return 0.0; }
  }

  velocity_t getVelocity(time_t time)
  {
    if (0_s < time && time < m_accelTime) { return m_maxAccel * time; }
    else if (m_accelTime < time && time < m_accelTime + m_coastTime)
    {
      return m_maxVelocity;
    }
    else if (m_accelTime + m_coastTime < time && time < m_totalTime)
    {
      return m_maxVelocity - m_maxDecel * (time - m_accelTime - m_coastTime);
    }
    else { return 0.0; }
  }

  acceleration_t getAcceleration(time_t time)
  {
    if (time_t(0) < time && time < m_accelTime) { return m_maxAccel; }
    else if (m_accelTime < time && time < m_accelTime + m_coastTime) { return 0.0; }
    else if (m_accelTime + m_coastTime < time && time < m_totalTime)
    {
      return -m_maxDecel;
    }
    else { return 0.0; }
  }
};
}  // namespace lib2131::utilities