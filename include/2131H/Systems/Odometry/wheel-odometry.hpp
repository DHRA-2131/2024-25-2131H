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
#include "2131H/Utilities/console.hpp"
#include "2131H/Utilities/pose.hpp"
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
  double m_prevAngle = 0;  //! USES RADIANS INTERNALLY

  // Current Wheel Values
  double m_verticalDistance = 0;
  double m_horizontalDistance = 0;
  double m_angle = 0;

  // Odometry Wheel Offset
  const double m_verticalOffset;
  const double m_horizontalOffset;

  Pose debugPose = Pose(0, 0, 0);
  double debugDouble = 0;

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
   * @param radians is Measure of angle in radians? (True by default)
   * @return double chord length
   */
  double _calculateChordLength(double length, double offset, double angle, bool radians = true)
  {
    // Technically when using wheel only odometry this will be calculated eventually
    // However, when using the IMU, the angle change may not be 0.0 when the length is 0.0
    // So to skip some math we can just return 0.0 and it should be more accurate too.
    if (length == 0) { return 0.0; }

    // Convert to radians
    if (!radians) { angle = angle * M_PI / 180.0; }

    // If there is no angle change we can just return the length traveled
    if (angle == 0) { return length; }

    // Otherwise, Calculate the arc length
    const double radius = offset + (length / angle);
    double out = 2.0 * radius * std::sin(angle / 2.0);
    return out;
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
    if (usingIMU && std::isinf(m_sensors.inertial->get_heading()))
    {
      return Pose(0, 0, 0);  // Inertial is DC or Calibrating so don't calculate odom
    }

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
    else
    {
      m_verticalDistance = 0;
      Console.log(Logger::BG_Red, "NEEDS VERTICAL WHEELS");
    }

    // If there is a horizontal wheel, then update horizontal change
    if (this->m_sensors.horizontal != nullptr)
    {
      m_prevHorizontal = m_horizontalDistance;  // Reset Previous Position
      m_horizontalDistance = this->m_sensors.horizontal->getDistanceTraveled();
    }

    if (usingIMU)  // Use the inertial to calculate heading delta
    {
      m_prevAngle = m_angle;  // Reset Previous Position
      m_angle = this->m_sensors.inertial->get_heading() / 180.0 * M_PI;
      // Get Heading from Inertial and convert to radians
    }
    else  // Use wheels to calculate heading delta
    {
      Console.log(Logger::BG_Red, "WHEEL HEADING CHANGE");
      // TODO: WHEEL HEADING CHANGE
    }

    // Calculate change from last loop
    double deltaVertical = m_verticalDistance - m_prevVertical;
    double deltaHorizontal = m_horizontalDistance - m_prevHorizontal;
    double deltaAngle = m_angle - m_prevAngle;

    // Approximate wheel path as an Arc and calculate chord length to calculate Local Change
    double localX = this->_calculateChordLength(deltaVertical, m_verticalOffset, deltaAngle);
    double localY = this->_calculateChordLength(deltaHorizontal, m_horizontalOffset, deltaAngle);

    // Average theta change (for rotating, don't want to rotate by start or end theta)
    double avgAngle = currentPose.getTheta(true) - deltaAngle / 2;

    // Convert to a local Pose
    Pose localPose(localX, localY, deltaAngle, true);
    // return (Rotate local Pose to global coordinate system and add to current Pose)
    auto out = (localPose.rotate(avgAngle, true));
    out.setTheta(deltaAngle, true);  // Set Angle

    return out;
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