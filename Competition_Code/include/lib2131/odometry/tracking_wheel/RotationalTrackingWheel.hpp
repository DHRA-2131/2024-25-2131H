#pragma once
#include <memory>

#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "pros/rotation.hpp"

namespace lib2131::odometry::trackingWheel
{
class RotationalTrackingWheel : public AbstractTrackingWheel
{
 private:  // Pointers
  std::unique_ptr<pros::Rotation> m_pEncoder;

 public:  // Constructors
  RotationalTrackingWheel(int8_t port, const float offset, const float wheelDiameter,
                          const float ratio, double distance = 0)
      : AbstractTrackingWheel(offset, wheelDiameter, ratio, distance),
        m_pEncoder(std::make_unique<pros::Rotation>(port))
  {
    m_pEncoder->set_data_rate(5);
  }

 public:  // Overriden methods
  void tareSensor() override { m_pEncoder->reset_position(); }
  double getRaw() override { return m_pEncoder->get_position() / 100.0; }
};
}  // namespace lib2131::odometry::trackingWheel