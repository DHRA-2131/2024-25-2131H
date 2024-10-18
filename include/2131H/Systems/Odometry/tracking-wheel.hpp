/**
 * @file tracking-wheel.hpp
 * @author Andrew Hilton (2131H)
 * @brief Class Declaration for the TrackingWheel class
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cmath>

#include "2131H/Utilities/average.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

using namespace Utilities;

namespace Systems
{
class TrackingWheel
{
 private:  // === Member Variables === //
  pros::adi::Encoder* m_pEncoder = nullptr;
  pros::v5::Rotation* m_pRotation = nullptr;
  pros::v5::MotorGroup* m_pMotorGroup = nullptr;

  double m_wheelDiameter;
  double m_ratio;
  double m_offset;

 public:  // === Constructors === //
  /**
   * @brief Construct a new Tracking Wheel using a pros::adi::Encoder
   *
   * @param pEncoder Pointer to Encoder
   * @param wheelDiameter
   * @param ratio Ratio from wheel to encoder
   */
  TrackingWheel(pros::adi::Encoder* pEncoder, double wheelDiameter, double ratio = 1)
      : m_pEncoder(pEncoder), m_wheelDiameter(wheelDiameter), m_ratio(ratio)
  {
  }
  /**
   * @brief Construct a new Tracking Wheel using a pros::v5::Rotation
   *
   * @param pRotation Pointer to Rotation
   * @param wheelDiameter
   * @param ratio Ratio from wheel to Rotational Sensor
   */
  TrackingWheel(pros::v5::Rotation* pRotation, double wheelDiameter, double ratio = 1)
      : m_pRotation(pRotation), m_wheelDiameter(wheelDiameter), m_ratio(ratio)
  {
  }

  /**
   * @brief Construct a new Tracking Wheel using a pros::v5::MotorGroup
   *
   * @param pMotorGroup Pointer to MotorGroup
   * @param wheelDiameter
   * @param rpm RPM of chassis
   */
  TrackingWheel(pros::v5::MotorGroup* pMotorGroup, double wheelDiameter, double rpm)
      : m_pMotorGroup(pMotorGroup), m_wheelDiameter(wheelDiameter), m_ratio(rpm)
  {
  }

 public:  // === Functions === //
  /**
   * @brief Get the Distance Traveled by Tracking Wheel
   *
   * @return double Distance
   */
  double getDistanceTraveled()
  {
    // If using the Encoder
    if (this->m_pEncoder != nullptr)
    {
      // ((Encoder Value [deg]) * (Wheel Diameter [in]) * Pi / 360 [deg]) / (Gear Ratio)
      return (float(this->m_pEncoder->get_value()) * this->m_wheelDiameter * M_PI / 360) /
             this->m_ratio;
    }
    // If using the Rotation Sensor
    else if (this->m_pRotation != nullptr)
    {
      // ((Encoder Value [cent-deg]) * (Wheel Diameter [in]) * Pi / 36000 [cent-deg]) / (Gear Ratio)
      return (float(this->m_pRotation->get_position()) * this->m_wheelDiameter * M_PI / 36000) /
             this->m_ratio;
    }
    // If using a MotorGroup
    else if (this->m_pMotorGroup != nullptr)
    {
      // Set encoder unit
      this->m_pMotorGroup->set_encoder_units_all(pros::MotorEncoderUnits::degrees);

      // Grab Motor Info
      std::vector<pros::MotorGears> gearsets = this->m_pMotorGroup->get_gearing_all();
      std::vector<double> positions = this->m_pMotorGroup->get_position_all();

      // Calculated Distance off of the motor Positions
      std::vector<float> distances;
      for (int i = 0; i < this->m_pMotorGroup->size(); i++)
      {
        float in;  // Internal Cartridge
        switch (gearsets[i])
        {
          case pros::MotorGears::red:
            in = 100;  // RPM of 100
            break;
          case pros::MotorGears::green:
            in = 200;  // RPM of 200
            break;
          case pros::MotorGears::blue:
            in = 600;  // RPM of 600
            break;
          default:
            in = 200;  // RPM of 200
            break;
        }

        // (Positions [deg] / 360.0 [Deg])* (Circumference [In]) * (chassis [RPM] / Gearset [RPM])
        distances.push_back((positions[i] / 360.0) * (this->m_ratio / in) *
                            (this->m_wheelDiameter * M_PI));
      }
      // Avg all motor distance
      return avg(distances);
    }
    // Not using any sensor (Should never happen)
    else { return 0; }
  }
  /**
   * @brief Get the Offset of Tracking Wheel
   *
   * @return double offset
   */
  double getOffset() { return m_offset; }
};
}  // namespace Systems