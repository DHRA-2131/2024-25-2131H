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
 * @brief Construct a new Odometry object using 3 Wheels, Put nullptr to exclude a
 * wheel.
 *
 * @param LeftWheel Pointer to Left trackingWheel object
 * @param RightWheel Pointer to Right trackingWheel object
 * @param RearWheel Pointer to Rear trackingWheel object
 */
Odometry::Odometry(TrackingWheel* LeftWheel, TrackingWheel* RightWheel,
                   TrackingWheel* RearWheel, pros::v5::IMU* Inertial)
    : leftWheel(LeftWheel),
      rightWheel(RightWheel),
      rearWheel(RearWheel),
      inertial(Inertial),
      currentState()
{
  leftExists = (leftWheel != nullptr);
  rightExists = (rightWheel != nullptr);
  rearExists = (rearWheel != nullptr);
  inertialExists = (rearWheel != nullptr);

  if (leftExists) leftWheel->reset();
  if (rightExists) rightWheel->reset();
  if (rearExists) rearWheel->reset();
}

/**
 * @brief Get the Robot State object
 *
 * @return RobotState currentState
 */
RobotState Odometry::getRobotState() const { return currentState; }

/**
 * @brief Get the Robot State object
 *
 * @return RobotState currentState
 */
void Odometry::setRobotState(RobotState newState) { currentState = newState; }

/**
 * @brief Update Odometry Class
 *
 * @param dTime Change in time between updates
 */
void Odometry::update(double dTime)
{
  // Get Deltas
  if (leftExists) dLeftDist = lastLeftDist - leftWheel->getDistanceTraveled();
  if (rightExists) dRightDist = lastRightDist - rightWheel->getDistanceTraveled();
  if (rearExists) dRearDist = lastRearDist - rearWheel->getDistanceTraveled();

  // FINDING THETA
  // Using Odom Wheels
  if (leftExists && rightExists)
  {
    if (leftWheel->getOffset() - rightWheel->getOffset() == 0)
    {
      ;  // TODO: Set-up as error
    }
    else
    {
      dTheta.setTheta(
          (dLeftDist - dRightDist) / (leftWheel->getOffset() - rightWheel->getOffset()),
          false);
    }
  }
  // Use Inertial for theta
  else if (inertialExists)
  {
    dTheta = lastTheta - Angle(360 - inertial->get_heading(), true);
  }
  else  // This should never happen
  {
    ;  // TODO: Set-Up as error
  }

  Angle avgTheta = Theta - dTheta / 2;
  Theta -= dTheta;

  // Local Change in X and Y
  double localX, localY;
  if (dTheta.getRadians() == 0)
  {
    localX = dRearDist;
    if (leftExists)
      localY = dLeftDist;
    else if (rightExists)
      localY = dRightDist;
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

  RobotState lastState = currentState;

  // Set Robot Actual Position
  Vector3<double, double, Angle> dGlobalPosition(
      localY * sin(avgTheta.getRadians()) + localX * -cos(avgTheta.getRadians()),
      localY * cos(avgTheta.getRadians()) + localX * sin(avgTheta.getRadians()), dTheta);

  // Update Robot State
  currentState.position += dGlobalPosition;
  currentState.velocity = (currentState.position - lastState.position) * (1000 / dTime);
  currentState.acceleration =
      (currentState.velocity - lastState.velocity) * (1000 / dTime);

  // Update "Last" Data
  if (leftExists) lastLeftDist = leftWheel->getDistanceTraveled();
  if (rightExists) lastRightDist = rightWheel->getDistanceTraveled();
  if (rearExists) lastRearDist = rearWheel->getDistanceTraveled();
  lastTheta = Theta;
}

}  // namespace lib2131