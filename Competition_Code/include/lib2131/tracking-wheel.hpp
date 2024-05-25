/**
 * @file tracking-wheel.hpp
 * @author LemLib
 * (https://github.com/LemLib/LemLib/blob/6d9e40d8e65e8326c8a87b4f30ef8724b0b5421b/src/lemlib/chassis/trackingWheel.cpp)
 * @brief Tracking Wheel class taken from LemLib, changed as to have better
 * naming. Copyright belongs to LemLib and it's contributors.
 * @version 0.1
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cmath>

#include "lib2131/utilities.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace lib2131
{

/**
 * @brief trackingWheel Class, Modified from LemLib
 *
 */
class trackingWheel
{
 private:
  float diameter;
  float offset;
  float driveRpm;
  float gearRatio;

  pros::adi::Encoder *encoder = nullptr;
  pros::v5::Rotation *rotation = nullptr;
  pros::v5::MotorGroup *motors = nullptr;

 public:
  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param Encoder Pointer to (ADI) Encoder
   * @param Diameter Wheel Size in inches
   * @param Offset Offset from Tracking Center
   * @param GearRatio Gear Ratio to sensor
   */
  trackingWheel(pros::adi::Encoder *Encoder, float Diameter, float Offset,
                float GearRatio = 1);

  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param Encoder Pointer to (Rotational) Encoder
   * @param Diameter Wheel Size in inches
   * @param Offset Offset from Tracking Center
   * @param GearRatio Gear Ratio to sensor
   */
  trackingWheel(pros::v5::Rotation *Encoder, float Diameter, float Offset,
                float GearRatio = 1);

  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param Motors Pointer to (Motor) Encoder(s)
   * @param Diameter Wheel size in inches
   * @param Offset Offset from Tracking Center
   * @param DriveRpm RPM at the wheels on drive
   */
  trackingWheel(pros::v5::MotorGroup *Motors, float Diameter, float Offset,
                float DriveRpm);

  /**
   * @brief Resets Odom Wheel
   *
   */
  void reset();

  /**
   * @brief Get the Distance Traveled by trackingWheel
   *
   * @return float Distance
   */
  float getDistanceTraveled();

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