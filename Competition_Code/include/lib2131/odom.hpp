/**
 * @file odom.hpp
 * @author Andrew Hilton (2131H)
 * @brief File for Odometry Class. Allows for full tracking of robot.
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "angle.hpp"
#include "lib2131/angle.hpp"
#include "lib2131/robot-state.hpp"
#include "lib2131/tracking-wheel.hpp"
#include "pros/imu.hpp"

namespace lib2131
{

/**
 * @brief Odometry Class. Provides full tracking of Robot.
 *
 */
class Odometry
{
 private:  // Lib2131
  TrackingWheel *leftWheel;
  TrackingWheel *rightWheel;
  TrackingWheel *rearWheel;

 private:  // Pros
  pros::v5::IMU *inertial;
  RobotState currentState;

 private:  // Angles
  Angle Theta;
  Angle lastTheta;
  Angle dTheta;

 private:  // Odometry Types
  const bool leftExists, rightExists, rearExists, inertialExists;
  bool calibrated;

 public:  // Constructors
  /**
   * @brief Construct a new odometry object using 3 Wheels, Put nullptr to
   * exclude a wheel.
   *
   * @param LeftWheel Pointer to Left trackingWheel object
   * @param RightWheel Pointer to Right trackingWheel object
   * @param RearWheel Pointer to Rear trackingWheel object
   * @param Inertial Pointer to Pros IMU object
   */
  Odometry(TrackingWheel *LeftWheel, TrackingWheel *RightWheel, TrackingWheel *RearWheel,
           pros::v5::IMU *Inertial);

 public:  // Getters & Setters
  /**
   * @brief Get the Robot State object
   *
   * @return RobotState currentState
   */
  RobotState getRobotState() const;

  /**
   * @brief Set the RobotState of Odometry Object
   *
   * @param newState New RobotState of Odometry Object
   */
  void setRobotState(RobotState newState);

 public:  // Functions
  /**
   * @brief Update Odometry Class
   *
   * @param dTime Change in time between updates
   */
  void update(double dTime);

 private:
  /**
   * @brief Calculate the Chord Length of a Arc
   *
   * @param length Length of Arc
   * @param offset Tracking Wheel Offset
   * @param theta Measure of Arc
   * @return double Chord Length
   */
  double _calculateChordLength(double length, double offset, Angle theta);
};
}  // namespace lib2131