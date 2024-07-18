#pragma once
#include <memory>

#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "pros/rotation.hpp"

namespace lib2131::odometry::trackingWheel
{
using namespace units::literals;
class RotationalTrackingWheel : public AbstractTrackingWheel
{
  using distance_t = units::length::inch_t;

 private:  // Pointers
  std::unique_ptr<pros::Rotation> m_pEncoder;

 public:  // Constructors
  RotationalTrackingWheel(int8_t port, distance_t offset, distance_t wheelDiameter,
                          const float ratio, distance_t distance = 0_in)
      : AbstractTrackingWheel(offset, wheelDiameter, ratio, distance),
        m_pEncoder(std::make_unique<pros::Rotation>(port))
  {
    m_pEncoder->set_data_rate(5);
  }

 public:  // Overriden methods
  void tareSensor() override { m_pEncoder->reset_position(); }
  units::angle::degree_t getRaw() override
  {
    return units::angle::degree_t(m_pEncoder->get_position() / 100.0);
  }
};
}  // namespace lib2131::odometry::trackingWheel