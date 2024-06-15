/**
 * @file tracking-wheel.hpp
 * @author LemLib
 * (https://github.com/LemLib/LemLib/blob/6d9e40d8e65e8326c8a87b4f30ef8724b0b5421b/src/lemlib/chassis/trackingWheel.cpp)
 * @brief Tracking Wheel Source Code. (Is Modified, Original by LemLib)
 * @version 0.1
 * @date 2024-05-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lib2131/tracking-wheel.hpp"

#include "lib2131/utilities.hpp"
#include "pros/abstract_motor.hpp"

namespace lib2131
{

/**
 * @brief Construct a new tracking Wheel object
 *
 * @param Encoder Pointer to (ADI) Encoder
 * @param Diameter Wheel Size in inches
 * @param Offset Offset from Tracking Center
 * @param GearRatio Gear Ratio to sensor
 */
TrackingWheel::TrackingWheel(pros::adi::Encoder *Encoder, float Diameter, float Offset,
                             float GearRatio)
    : m_distance(0),
      m_init(1),
      m_deltaDistance(0),
      m_lastDistance(0),
      m_pEncoder(Encoder),
      m_diameter(Diameter),
      m_offset(Offset),
      m_gearRatio(GearRatio)
{
  this->reset();
}

/**
 * @brief Construct a new tracking Wheel object
 *
 * @param Encoder Pointer to (Rotational) Encoder
 * @param Diameter Wheel Size in inches
 * @param Offset Offset from Tracking Center
 * @param GearRatio Gear Ratio to sensor
 */
TrackingWheel::TrackingWheel(pros::v5::Rotation *Encoder, float Diameter, float Offset,
                             float GearRatio)
    : m_distance(0),
      m_init(1),
      m_deltaDistance(0),
      m_lastDistance(0),
      m_pRotation(Encoder),
      m_diameter(Diameter),
      m_offset(Offset),
      m_gearRatio(GearRatio)
{
  this->reset();
}

/**
 * @brief Construct a new tracking Wheel object
 *
 * @param Motors Pointer to Motor Encoder(s)
 * @param Diameter Wheel size in inches
 * @param Offset Offset from Tracking Center
 * @param DriveRpm RPM at the wheels on drive
 */
TrackingWheel::TrackingWheel(pros::v5::MotorGroup *Motors, float Diameter, float Offset,
                             float DriveRpm)
    : m_distance(0),
      m_init(1),
      m_deltaDistance(0),
      m_lastDistance(0),
      m_pMotors(Motors),
      m_diameter(Diameter),
      m_offset(Offset),
      m_driveRpm(DriveRpm)
{
  this->reset();
}

/**
 * @brief Resets Odom Wheel
 *
 */
void TrackingWheel::reset()
{
  if (this->m_pEncoder != nullptr) this->m_pEncoder->reset();
  if (this->m_pRotation != nullptr) this->m_pRotation->reset_position();
  if (this->m_pMotors != nullptr) this->m_pMotors->tare_position_all();
}

/**
 * @brief Get the Distance Traveled by TrackingWheel
 *
 * @return float Distance
 */
float TrackingWheel::getDistance() { return m_distance; }
float TrackingWheel::getDeltaDistance() { return m_deltaDistance; }
float TrackingWheel::getLastDistance() { return m_lastDistance; }

/**
 * @brief Update Tracking Wheel
 *
 */
void TrackingWheel::update()
{
  if (this->m_pEncoder != nullptr)
  {
    m_distance = (float(this->m_pEncoder->get_value()) * this->m_diameter * M_PI / 360) /
                 this->m_gearRatio;
  }
  else if (this->m_pRotation != nullptr)
  {
    m_distance =
        (float(this->m_pRotation->get_position()) * this->m_diameter * M_PI / 36000) /
        this->m_gearRatio;
  }
  else if (this->m_pMotors != nullptr)
  {
    // Get each motor's GearSet
    std::vector<pros::v5::MotorGears> gearsets = this->m_pMotors->get_gearing_all();
    // Make sure all the encoders are using Revolutions
    this->m_pMotors->set_encoder_units_all(pros::MotorEncoderUnits::degrees);
    // Get Position of Motors
    std::vector<double> positions = this->m_pMotors->get_position_all();
    // Calculate Distances for each motor
    std::vector<float> distances;
    for (int i = 0; i < this->m_pMotors->size(); i++)
    {
      float rpm;
      switch (gearsets[i])
      {
        case pros::v5::MotorGears::red:
          rpm = 100.0;
          break;
        case pros::v5::MotorGears::green:
          rpm = 200.0;
          std::cout << "GREEN MOTOR: " << i << std::endl;
          break;
        case pros::v5::MotorGears::blue:
          rpm = 600.0;
          break;
        default:
          break;
      }

      distances.push_back(positions[i] / 360.0 * (m_diameter * M_PI) *
                          (m_driveRpm / rpm));
    }
    // Distance = Average of each motor
    m_distance = average(distances);
  }
  else
  {
    m_distance = 0;  // Should never execute
  }

  if (m_init)
  {
    m_deltaDistance = 0;
    m_init = false;
  }
  else
  {
    // Update delta
    m_deltaDistance = m_distance - m_lastDistance;
  }

  // Update Last
  m_lastDistance = m_distance;
}

/**
 * @brief Get the Offset from Tracking Center
 *
 * @return float Offset
 */
float TrackingWheel::getOffset() { return this->m_offset; }

/**
 * @brief Get the Type of Odom
 *
 * @return bool isMotor?
 */
bool TrackingWheel::getType() { return (this->m_pMotors != nullptr); }
}  // namespace lib2131