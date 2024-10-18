/**
 * @file wheel-odometry.hpp
 * @author Andrew Hilton (2131H)
 * @brief Class Declaration for the Wheel Odometry Class
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <math.h>

#include <cmath>

#include "2131H/Systems/Odometry/abstract-odometry.hpp"
#include "2131H/Systems/Odometry/tracking-wheel.hpp"
#include "pros/imu.hpp"

namespace Systems
{

struct TrackingSensors
{
  TrackingWheel* vertical1 = nullptr;   // Vertical Odom Wheel
  TrackingWheel* vertical2 = nullptr;   // Second Vertical Odom Wheel
  TrackingWheel* horizontal = nullptr;  // Horizontal Wheel

  pros::IMU* inertial = nullptr;  // Inertial Sensor (For Heading)
};

class WheelOdometry : public AbstractOdometry
{
 private:  // === Member Variables  == //
  // Pointers to sensors
  TrackingSensors m_sensors;

  // Are we using an Inertial Sensor?
  bool usingIMU;

  // Previous wheel values
  double m_prevVertical;
  double m_prevHorizontal;
  double m_prevAngle;  //! USES RADIANS INTERNALLY

  // Current Wheel Values
  double m_verticalDistance = 0;
  double m_horizontalDistance = 0;
  double m_angle = 0;

  // Odometry Wheel Offset
  const double m_verticalOffset;
  const double m_horizontalOffset;

 public:  // === Constructors == //
  /**
   * @brief Construct a Odometry Instance
   * @note Two Vertical wheels are needed if there is no inertial
   * @param sensors Sensors available to Odometry instance
   */
  WheelOdometry(TrackingSensors sensors)
      : m_sensors(sensors),
        usingIMU(sensors.inertial != nullptr),
        m_verticalOffset(sensors.vertical1 != nullptr ? sensors.vertical1->getOffset()
                                                      : sensors.vertical2->getOffset()),
        m_horizontalOffset(sensors.horizontal != nullptr ? sensors.horizontal->getOffset() : 0),
        m_prevVertical(sensors.vertical1 != nullptr ? sensors.vertical1->getDistanceTraveled()
                                                    : sensors.vertical2->getDistanceTraveled()),
        m_prevHorizontal(sensors.horizontal != nullptr ? sensors.horizontal->getDistanceTraveled()
                                                       : 0)
  {
  }

 private:  // === Member Functions == //
  /**
   * @brief Calculates the chord length of a Arc given arc parameters
   *
   * @param length Arc Length
   * @param offset Offset from arc length
   * @param angle Measure of angle
   * @param Radians is Measure of angle in Radians? (True by default)
   * @return double chord length
   */
  double _calculateChordLength(double length, double offset, double angle, bool Radians = true)
  {
    if (!Radians) { angle = angle * M_PI / 180; }
    if (angle == 0) { return length; }
    const double radius = offset + (length / angle);
    return 2 * radius * std::sin(angle / 2);
  }

 public:  // === Functions == //
  /**
   * @brief Updates pose with calculated change in location
   *
   * @param currentPose Current Position as a Pose
   * @return Pose new Position
   */
  Pose updatePose(Pose& currentPose) override
  {
    if (usingIMU && !std::isinf(m_sensors.inertial->get_heading()))
    {
      // Figure out which wheel is going to be used for local change
      // Attempt to use Vertical 1
      if (this->m_sensors.vertical1 != nullptr)
      {
        m_prevVertical = m_verticalDistance;  // Reset Previous Position
        m_verticalDistance = this->m_sensors.vertical1->getDistanceTraveled();
      }
      // If that fails, Attempt to use Vertical 2
      else if (this->m_sensors.vertical2 != nullptr)
      {
        m_prevVertical = m_verticalDistance;  // Reset Previous Position
        m_verticalDistance = this->m_sensors.vertical2->getDistanceTraveled();
      }
      // Need vertical wheels so this should never happen
      else { m_verticalDistance = 0; }

      // If there is a horizontal wheel, then update horizontal change
      if (this->m_sensors.horizontal != nullptr)
      {
        m_prevHorizontal = m_horizontalDistance;  // Reset Previous Position
        m_horizontalDistance = this->m_sensors.horizontal->getDistanceTraveled();
      }

      // If using the inertial to calculate heading
      if (usingIMU)
      {
        m_prevAngle = m_angle;  // Reset Previous Position
        // Get Heading from Inertial and convert to radians
        m_angle = this->m_sensors.inertial->get_heading() / 180.0 * M_PI;
      }
      else
      {  // TODO: Implement Wheel based Angle Calculation
      }

      // Calculate change from last loop
      double deltaVertical = m_verticalDistance - m_prevVertical;
      double deltaHorizontal = m_horizontalDistance - m_prevHorizontal;
      double deltaAngle = m_angle - m_prevAngle;

      // Average theta change (for rotating, don't want to rotate by start or end theta)
      double avgAngle = m_prevAngle + deltaAngle / 2;

      // Approximate wheel path as an Arc and calculate chord length to calculate Local Change
      double localX = this->_calculateChordLength(deltaVertical, m_verticalOffset, avgAngle);
      double localY = this->_calculateChordLength(deltaHorizontal, m_horizontalOffset, avgAngle);

      // Convert to a local Pose
      Pose localPose(localX, localY, m_angle, true);
      // return (Rotate local Pose to global coordinate system and add to current Pose)
      auto out = currentPose + localPose.rotate(avgAngle, true);
      out.setTheta(m_angle, true);  // Set Angle

      return out;
    }
    else { return currentPose; }
  }

  /**
   * @brief Calibrate Inertial if it exists
   *
   * @param blocking
   */
  void init(bool blocking) override
  {
    if (usingIMU) { m_sensors.inertial->reset(blocking); }
  }
};
}  // namespace Systems