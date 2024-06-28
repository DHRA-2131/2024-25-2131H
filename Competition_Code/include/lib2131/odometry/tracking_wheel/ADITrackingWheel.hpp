#pragma once
#include <memory>

#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "pros/adi.hpp"

namespace lib2131::odometry::trackingWheel
{
using namespace units::literals;
class ADITrackingWheel : public AbstractTrackingWheel
{
  using distance_t = units::length::inch_t;

 private:  // Pointers
  std::unique_ptr<pros::adi::Encoder> m_pEncoder;

 public:  // Constructors
  ADITrackingWheel(int8_t port, distance_t offset, distance_t wheelDiameter,
                   const float ratio, distance_t distance = 0_in)
      : AbstractTrackingWheel(offset, wheelDiameter, ratio, distance),
        m_pEncoder(
            std::make_unique<pros::adi::Encoder>(abs(port), abs(port) + 1, port < 0))
  {
  }

 public:  // Overriden methods
  void tareSensor() override { m_pEncoder->reset(); }
  units::angle::degree_t getRaw() override
  {
    return units::angle::degree_t(m_pEncoder->get_value());
  }
};
}  // namespace lib2131::odometry::trackingWheel