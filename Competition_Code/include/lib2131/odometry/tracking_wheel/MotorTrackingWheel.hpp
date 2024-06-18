#pragma once
#include <memory>

#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "lib2131/utilities/Average.hpp"
#include "pros/motor_group.hpp"

namespace lib2131::odometry::trackingWheel
{
class MotorTrackingWheel : public AbstractTrackingWheel
{
 private:  // Pointers
  std::shared_ptr<pros::v5::MotorGroup> m_pEncoder;
  const float m_rpm;

 public:  // Constructors
  MotorTrackingWheel(const std::shared_ptr<pros::v5::MotorGroup>& motorGroup,
                     const float offset, const float wheelDiameter, const float rpm,
                     double distance = 0)
      : AbstractTrackingWheel(offset, wheelDiameter, 1, distance),
        m_pEncoder(std::move(motorGroup)),
        m_rpm(rpm)
  {
    m_pEncoder->set_encoder_units(pros::MotorUnits::degrees);
  }

 public:  // Overriden methods
  void tareSensor() override { m_pEncoder->tare_position_all(); }
  double getRaw() override
  {  // Get each motor's GearSet
    std::vector<pros::v5::MotorGears> gearsets = this->m_pEncoder->get_gearing_all();
    // Make sure all the encoders are using Revolutions
    this->m_pEncoder->set_encoder_units_all(pros::MotorEncoderUnits::degrees);
    // Get Position of Motors
    std::vector<double> positions = this->m_pEncoder->get_position_all();
    // Calculate Distances for each motor
    std::vector<float> distances;
    for (int i = 0; i < this->m_pEncoder->size(); i++)
    {
      float gearsetRpm;
      switch (gearsets[i])
      {
        case pros::v5::MotorGears::red:
          gearsetRpm = 100.0;
          break;
        case pros::v5::MotorGears::green:
          gearsetRpm = 200.0;
          break;
        case pros::v5::MotorGears::blue:
          gearsetRpm = 600.0;
          break;
        default:
          break;
      }

      distances.push_back(positions[i] / gearsetRpm);
    }
    // Distance = Average of each motor
    return lib2131::utilities::average(distances) * m_rpm;
  }
};
}  // namespace lib2131::odometry::trackingWheel