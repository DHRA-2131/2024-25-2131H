#pragma once
#include <cmath>

#include "lib2131/utilities/Units.h"

namespace lib2131::odometry::trackingWheel
{
using namespace units::literals;

class AbstractTrackingWheel
{
  using distance_t = units::length::inch_t;

 private:  // Constant Variables
  distance_t m_offset;
  distance_t m_wheelDiameter;
  const float m_ratio;

 private:  // Non Const Variables
  distance_t m_distance;
  distance_t m_distanceLast;
  distance_t m_deltaDistance;

 public:
  AbstractTrackingWheel(distance_t offset, distance_t wheelDiameter, const float ratio,
                        distance_t distance = 0_in)
      : m_offset(offset),
        m_wheelDiameter(wheelDiameter),
        m_ratio(ratio),
        m_distance(distance)
  {
  }

  distance_t getDistance() { return m_distance; }
  distance_t getDeltaDistance() { return m_deltaDistance; }

  distance_t getOffset() { return m_offset; }

  void update()
  {
    //* this->getRaw should be in degrees
    // Revolutions * Circumference / Ratio
    m_distance = this->getRaw() / 360.0 * m_wheelDiameter * M_PI / m_ratio;
    // Find Delta
    m_deltaDistance = m_distance - m_distanceLast;
    // Update Last
    m_distanceLast = m_distance;
  }

  void reset()
  {
    this->tareSensor();
    this->m_distance = distance_t(0);
    this->m_deltaDistance = distance_t(0);
    this->m_distanceLast = distance_t(0);
  }

  virtual void tareSensor() = 0;
  virtual units::angle::degree_t getRaw() = 0;
};
}  // namespace lib2131::odometry::trackingWheel