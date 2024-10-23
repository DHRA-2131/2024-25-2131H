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
 private:                                     // === Member Variables === //
  pros::adi::Encoder* m_pEncoder = nullptr;   // Encoder (Cortex)
  pros::v5::Rotation* m_pRotation = nullptr;  // Encoder (Rotational V5)

  // Motor Variables
  pros::v5::MotorGroup* m_pMotorGroup = nullptr;  // Encoder (V5 Motor)
  std::vector<bool> m_motorDCs;                   // All Motor DCs
  double m_lastAvgValue;                          // Last Average of motors

  double m_wheelDiameter;  // Wheel Diameter (In)
  double m_ratio;          // Ratio / RPM from sensor to Tracking wheel
  double m_offset;         // Offset from tracking center (In)

 public:  // === Constructors === //
  /**
   * @brief Construct a new Tracking Wheel using a pros::adi::Encoder
   *
   * @param pEncoder Pointer to Encoder
   * @param offset Offset from Tracking center
   * @param wheelDiameter Wheel Diameter
   * @param ratio Ratio from wheel to encoder
   */
  TrackingWheel(pros::adi::Encoder* pEncoder, double offset, double wheelDiameter, double ratio = 1)
      : m_pEncoder(pEncoder), m_offset(offset), m_wheelDiameter(wheelDiameter), m_ratio(ratio)
  {
  }
  /**
   * @brief Construct a new Tracking Wheel using a pros::v5::Rotation
   *
   * @param pRotation Pointer to Rotation
   * @param offset Offset from Tracking center
   * @param wheelDiameter Wheel Diameter
   * @param ratio Ratio from wheel to Rotational Sensor
   */
  TrackingWheel(pros::v5::Rotation* pRotation, double offset, double wheelDiameter,
                double ratio = 1)
      : m_pRotation(pRotation), m_offset(offset), m_wheelDiameter(wheelDiameter), m_ratio(ratio)
  {
  }

  /**
   * @brief Construct a new Tracking Wheel using a pros::v5::MotorGroup
   *
   * @param pMotorGroup Pointer to MotorGroup
   * @param offset Half the track width
   * @param wheelDiameter wheel Diameter
   * @param rpm RPM of chassis
   */
  TrackingWheel(pros::v5::MotorGroup* pMotorGroup, double offset, double wheelDiameter, double rpm)
      : m_pMotorGroup(pMotorGroup), m_offset(offset), m_wheelDiameter(wheelDiameter), m_ratio(rpm)
  {
    m_motorDCs.resize(m_pMotorGroup->size(), 0);
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
      std::vector<float> distances = {};
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
        double distance =
            (positions[i] / 360.0) * (this->m_ratio / in) * (this->m_wheelDiameter * M_PI);

        // Only add distance if it's *not* infinite
        if (!std::isinf(distance))
        {
          // If Motor hasn't been DCed then add it to list
          if (!m_motorDCs[i]) { distances.push_back(distance); }
        }
        else
        {
          m_motorDCs[i] = 1;  // Add DC to DC list
        }
      }

      // Avg all motor distance
      // Store value to be used in next loop
      m_lastAvgValue = avg(distances);
      return m_lastAvgValue;
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