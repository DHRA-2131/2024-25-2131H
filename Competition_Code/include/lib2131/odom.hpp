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
#include "lib2131/robot-state.hpp"
#include "lib2131/tracking-wheel.hpp"
#include "pros/imu.hpp"

namespace lib2131
{

/**
 * @brief Odometry Class. Provides full tracking of Robot.
 *
 */
class odometry
{
 private:
  trackingWheel* leftWheel = nullptr;
  trackingWheel* rightWheel = nullptr;
  trackingWheel* rearWheel = nullptr;

  pros::v5::IMU* inertial;

  angle Theta;

  double lastLeftDist;
  double lastRightDist;
  double lastRearDist;
  angle lastTheta;

  double dLeftDist;
  double dRightDist;
  double dRearDist;
  angle dTheta;

  bool leftExists, rightExists, rearExists, inertialExists;

  robotState currentState;

 public:
  /**
   * @brief Construct a new odometry object using 3 Wheels, Put nullptr to exclude a
   * wheel.
   *
   * @param LeftWheel Pointer to Left trackingWheel object
   * @param RightWheel Pointer to Right trackingWheel object
   * @param RearWheel Pointer to Rear trackingWheel object
   */
  odometry(trackingWheel* LeftWheel, trackingWheel* RightWheel, trackingWheel* RearWheel);

  /**
   * @brief Construct a new odometry object using 2 Wheels and a IMU, Put nullptr to
   * exclude a wheel.
   *
   * @param LeftWheel Pointer to Left trackingWheel object
   * @param RearWheel Pointer to Rear trackingWheel object
   * @param Inertial Pointer to a Pros::v5::IMU object
   */
  odometry(trackingWheel* LeftWheel, trackingWheel* RearWheel, pros::v5::IMU* Inertial);

  /**
   * @brief Get the Robot State object
   *
   * @return robotState currentState
   */
  robotState getRobotState();

  /**
   * @brief Update Odometry Class
   *
   * @param dTime Change in time between updates
   */
  void update(double dTime);
};
}  // namespace lib2131