/**
 * @file odom.cpp
 * @author Andrew Hilton (2131H)
 * @brief Odometry Source Code
 * @version 0.1
 * @date 2024-05-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lib2131/odom.hpp"

namespace lib2131
{

/**
 * @brief Construct a new odometry object using 3 Wheels, Put nullptr to exclude a
 * wheel.
 *
 * @param LeftWheel Pointer to Left trackingWheel object
 * @param RightWheel Pointer to Right trackingWheel object
 * @param RearWheel Pointer to Rear trackingWheel object
 */
odometry::odometry(trackingWheel* LeftWheel, trackingWheel* RightWheel,
                   trackingWheel* RearWheel)
    : leftWheel(LeftWheel), rightWheel(RightWheel), rearWheel(RearWheel), currentState()
{
  leftExists = (leftWheel != nullptr);
  rightExists = (rightWheel != nullptr);
  rearExists = (rearWheel != nullptr);
  inertialExists = (leftWheel != nullptr);
}

/**
 * @brief Construct a new odometry object using 2 Wheels and a IMU, Put nullptr to
 * exclude a wheel.
 *
 * @param LeftWheel Pointer to Left trackingWheel object
 * @param RearWheel Pointer to Rear trackingWheel object
 * @param Inertial Pointer to a Pros::v5::IMU object
 */
odometry::odometry(trackingWheel* LeftWheel, trackingWheel* RearWheel,
                   pros::v5::IMU* Inertial)
    : leftWheel(LeftWheel), rearWheel(RearWheel), inertial(Inertial), currentState()
{
  leftExists = (leftWheel != nullptr);
  rightExists = (rightWheel != nullptr);
  rearExists = (rearWheel != nullptr);
  inertialExists = (leftWheel != nullptr);
}

/**
 * @brief Get the Robot State object
 *
 * @return robotState currentState
 */
robotState odometry::getRobotState() { return currentState; }

/**
 * @brief Update Odometry Class
 *
 * @param dTime Change in time between updates
 */
void odometry::update(double dTime)
{
  // Get Deltas
  if (leftExists) dLeftDist = lastLeftDist - leftWheel->getDistanceTraveled();
  if (rightExists) dRightDist = lastRightDist - rightWheel->getDistanceTraveled();
  if (rearExists) dRearDist = lastRearDist - rearWheel->getDistanceTraveled();

  // FINDING THETA
  // Use Inertial for theta
  if (inertialExists)
  {
    Theta.setTheta(inertial->get_heading(), true);
    dTheta = lastTheta - Theta;
  }
  else  // Use Odom Wheels
  {
    // Both wheels needed
    if (leftExists && rightExists)
    {
      dTheta.setTheta(
          (dLeftDist - dRightDist) / (leftWheel->getOffset() - rightWheel->getOffset()),
          false);
      Theta += dTheta;
    }
    else  // This should never happen
    {
      ;  // TODO: Set-Up as error
    }
  }

  // Local Change in X and Y
  double localX, localY;
  if (dTheta.getRadians() == 0)
  {
    localX = dRearDist;
    if (leftExists)
      localY = dLeftDist;
    else if (rightExists)
      localY = dLeftDist;
    else
      ;  // TODO: Set-Up as error
  }
  else
  {
    if (leftExists)
    {
      localY = 2 * sin(dTheta.getRadians() / 2) *
               (dLeftDist / dTheta.getRadians() + leftWheel->getOffset());
    }
    else if (rightExists)
    {
      localY = 2 * sin(dTheta.getRadians() / 2) *
               (dLeftDist / dTheta.getRadians() + rightWheel->getOffset());
    }
    else
    {
      // TODO: Set-Up as error
    }

    if (rearExists)
    {
      localX = 2 * sin(dTheta.getRadians() / 2) *
               (dRearDist / dTheta.getRadians() + rearWheel->getOffset());
    }
  }

  robotState lastState = currentState;

  // Set Robot Actual Position
  Vector3<double, double, angle> dGlobalPosition(
      localY * sin(dTheta.getRadians()) + localX * -cos(dTheta.getRadians()),
      localY * cos(dTheta.getRadians()) + localX * sin(dTheta.getRadians()), dTheta);

  // Update Robot State
  currentState.position += dGlobalPosition;
  currentState.velocity = (currentState.position - lastState.position) * (1000 / dTime);
  currentState.acceleration =
      (currentState.velocity - lastState.velocity) * (1000 / dTime);

  // Update "Last" Data
  if (leftExists) lastLeftDist = leftWheel->getDistanceTraveled();
  if (rightExists) lastLeftDist = leftWheel->getDistanceTraveled();
  if (rearExists) lastLeftDist = leftWheel->getDistanceTraveled();
  lastTheta = Theta;
}

}  // namespace lib2131