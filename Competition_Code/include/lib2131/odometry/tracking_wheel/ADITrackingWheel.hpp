#pragma once
#include <memory>

#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "pros/adi.hpp"

namespace lib2131::odometry::trackingWheel
{
class ADITrackingWheel : public AbstractTrackingWheel
{
 private:  // Pointers
  std::unique_ptr<pros::adi::Encoder> m_pEncoder;

 public:  // Constructors
  ADITrackingWheel(int8_t port, const float offset, const float wheelDiameter,
                   const float ratio, double distance = 0)
      : AbstractTrackingWheel(offset, wheelDiameter, ratio, distance),
        m_pEncoder(
            std::make_unique<pros::adi::Encoder>(abs(port), abs(port) + 1, port < 0))
  {
  }

 public:  // Overriden methods
  void tareSensor() override { m_pEncoder->reset(); }
  double getRaw() override { return m_pEncoder->get_value(); }
};
}  // namespace lib2131::odometry::trackingWheel