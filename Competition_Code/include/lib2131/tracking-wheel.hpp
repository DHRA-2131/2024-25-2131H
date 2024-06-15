/**
 * @file tracking-wheel.hpp
 * @author Andrew Hilton (2131H)
 * @brief Tracking Wheel class
 * @version 0.1
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cmath>

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

namespace lib2131
{

/**
 * @brief TrackingWheel Class, Modified from LemLib
 *
 */
class TrackingWheel
{
 private:
  float m_diameter;
  float m_offset;
  float m_driveRpm;
  float m_gearRatio;

  bool m_init;
  float m_distance;
  float m_lastDistance;
  float m_deltaDistance;

  pros::adi::Encoder *m_pEncoder;
  pros::v5::Rotation *m_pRotation;
  pros::v5::MotorGroup *m_pMotors;

 public:
  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param encoder Pointer to (ADI) Encoder
   * @param diameter Wheel Size in inches
   * @param offset Offset from Tracking Center
   * @param gearRatio Gear Ratio to sensor
   */
  TrackingWheel(pros::adi::Encoder *encoder, float diameter, float offset,
                float gearRatio = 1);

  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param encoder Pointer to (Rotational) Encoder
   * @param diameter Wheel Size in inches
   * @param offset Offset from Tracking Center
   * @param gearRatio Gear Ratio to sensor
   */
  TrackingWheel(pros::v5::Rotation *encoder, float diameter, float offset,
                float gearRatio = 1);

  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param motors Pointer to (Motor) Encoder(s)
   * @param diameter Wheel size in inches
   * @param offset Offset from Tracking Center
   * @param driveRpm RPM at the wheels on drive
   */
  TrackingWheel(pros::v5::MotorGroup *motors, float diameter, float offset,
                float driveRpm);

  /**
   * @brief Resets Odom Wheel
   *
   */
  void reset();

  /**
   * @brief Get the Distance Traveled by TrackingWheel
   *
   * @return float Distance
   */
  float getDistance();

  /**
   * @brief Get the Change in Distance since last update
   *
   * @return float
   */
  float getDeltaDistance();

  /**
   * @brief Get the Distance from last update
   *
   * @return float
   */
  float getLastDistance();

  /**
   * @brief Update Tracking Wheel
   *
   */
  void update();

  /**
   * @brief Get the Offset from Tracking Center
   *
   * @return float Offset
   */
  float getOffset();

  /**
   * @brief Get the Type of Odom
   *
   * @return bool isMotor?
   */
  bool getType();
};
}  // namespace lib2131