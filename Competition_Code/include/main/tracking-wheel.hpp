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

#include "main/utilities.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

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
                float GearRatio = 1)
      : encoder(encoder), diameter(Diameter), offset(Offset), gearRatio(GearRatio)
  {
  }

  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param Encoder Pointer to (Rotational) Encoder
   * @param Diameter Wheel Size in inches
   * @param Offset Offset from Tracking Center
   * @param GearRatio Gear Ratio to sensor
   */
  trackingWheel(pros::v5::Rotation *Encoder, float Diameter, float Offset,
                float GearRatio = 1)
      : rotation(Encoder), diameter(Diameter), offset(Offset), gearRatio(GearRatio)
  {
  }

  /**
   * @brief Construct a new tracking Wheel object
   *
   * @param Motors Pointer to (Motor) Encoder(s)
   * @param Diameter Wheel size in inches
   * @param Offset Offset from Tracking Center
   * @param DriveRpm RPM at the wheels on drive
   */
  trackingWheel(pros::v5::MotorGroup *Motors, float Diameter, float Offset,
                float DriveRpm)
      : motors(Motors), diameter(Diameter), offset(Offset), driveRpm(DriveRpm)
  {
  }

  /**
   * @brief Resets Odom Wheel
   *
   */
  void reset()
  {
    if (this->encoder != nullptr) this->encoder->reset();
    if (this->rotation != nullptr) this->rotation->reset_position();
    if (this->motors != nullptr) this->motors->tare_position();
  }

  /**
   * @brief Get the Distance Traveled by trackingWheel
   *
   * @return float Distance
   */
  float getDistanceTraveled()
  {
    if (this->encoder != nullptr)
    {
      return (float(this->encoder->get_value()) * this->diameter * M_PI / 360) /
             this->gearRatio;
    }
    else if (this->rotation != nullptr)
    {
      return (float(this->rotation->get_position()) * this->diameter * M_PI / 36000) /
             this->gearRatio;
    }
    else if (this->motors != nullptr)
    {
      // get distance traveled by each motor
      std::vector<pros::v5::MotorGears> gearsets = this->motors->get_gearing_all();
      std::vector<double> positions = this->motors->get_position_all();
      std::vector<float> distances;
      for (int i = 0; i < this->motors->size(); i++)
      {
        float in;
        switch (gearsets[i])
        {
          case pros::v5::MotorGears::red:
            in = 100;
            break;
          case pros::v5::MotorGears::green:
            in = 200;
            break;
          case pros::MotorGears::blue:
            in = 600;
            break;
          default:
            in = 200;
            break;
        }
        distances.push_back(positions[i] * (diameter * M_PI) * (driveRpm / in));
      }
      return average(distances);
    }
    else
    {
      return 0;
    }
  }

  /**
   * @brief Get the Offset from Tracking Center
   *
   * @return float Offset
   */
  float getOffset() { return this->offset; }

  /**
   * @brief Get the Type of Odom
   *
   * @return bool isMotor?
   */
  bool getType()
  {
    if (this->motors != nullptr) return 1;
    return 0;
  }
};