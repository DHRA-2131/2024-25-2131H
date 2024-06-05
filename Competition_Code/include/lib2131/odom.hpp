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
#include "lib2131/Angle.hpp"
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
 private:
  TrackingWheel* leftWheel = nullptr;
  TrackingWheel* rightWheel = nullptr;
  TrackingWheel* rearWheel = nullptr;

  pros::v5::IMU* inertial;

  Angle Theta;

  double lastLeftDist;
  double lastRightDist;
  double lastRearDist;
  Angle lastTheta;

  double dLeftDist;
  double dRightDist;
  double dRearDist;
  Angle dTheta;

  bool leftExists, rightExists, rearExists, inertialExists;

  RobotState currentState;

 public:
  /**
   * @brief Construct a new odometry object using 3 Wheels, Put nullptr to exclude a
   * wheel.
   *
   * @param LeftWheel Pointer to Left trackingWheel object
   * @param RightWheel Pointer to Right trackingWheel object
   * @param RearWheel Pointer to Rear trackingWheel object
   * @param Inertial Pointer to Pros IMU object
   */
  Odometry(TrackingWheel* LeftWheel, TrackingWheel* RightWheel, TrackingWheel* RearWheel,
           pros::v5::IMU* Inertial);

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

  /**
   * @brief Update Odometry Class
   *
   * @param dTime Change in time between updates
   */
  void update(double dTime);
};
}  // namespace lib2131