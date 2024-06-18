#pragma once
#include <cmath>

namespace lib2131::odometry::trackingWheel
{

class AbstractTrackingWheel
{
 private:  // Constant Variables
  const float m_offset;
  const float m_wheelDiameter;
  const float m_ratio;

 private:  // Non Const Variables
  double m_distance;
  double m_distanceLast;
  double m_deltaDistance;

 public:
  AbstractTrackingWheel(const float offset, const float wheelDiameter, const float ratio,
                        double distance = 0)
      : m_offset(offset),
        m_wheelDiameter(wheelDiameter),
        m_ratio(ratio),
        m_distance(distance)
  {
  }

  double getDistance() { return m_distance; }
  double getDeltaDistance() { return m_deltaDistance; }

  const float getOffset() { return m_offset; }

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
    this->m_distance = 0;
    this->m_deltaDistance = 0;
    this->m_distanceLast = 0;
  }

  virtual void tareSensor() = 0;
  virtual double getRaw() = 0;
};
}  // namespace lib2131::odometry::trackingWheel