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

#include <cmath>

#include "tracking-wheel.hpp"

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
      currentState(),
      Theta({0, Inertial == nullptr}),
      lastTheta(0, Inertial == nullptr),
      leftExists(leftWheel != nullptr),
      rightExists(rightWheel != nullptr),
      rearExists(rearWheel != nullptr),
      inertialExists(inertial != nullptr)
{
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
  if (inertialExists)
  {
    if (std::isinf(inertial->get_heading())) { return; }
  }

  // Update Tracking Wheels
  if (leftExists) leftWheel->update();
  if (rightExists) rightWheel->update();
  if (rearExists) rearWheel->update();

  // Tracking Wheel Angle
  if (leftExists && rightExists && (!leftWheel->getType() || !leftWheel->getType()))
  {
    if (leftWheel->getOffset() - rightWheel->getOffset() != 0)
    {
      dTheta = Angle((leftWheel->getDeltaDistance() - rightWheel->getDeltaDistance()) /
                         (leftWheel->getOffset() - rightWheel->getOffset()),
                     false) *
               -1;
    }
    else
    {
      return;  // TODO: Throw Error
    }
  }
  // Inertial Angle
  else if (inertialExists && inertial->is_installed())
  {
    dTheta = lastTheta - Angle(inertial->get_heading() * -1, true);
  }
  else
  {
    return;  // TODO: Throw Error
  }

  double avgTheta = (Theta - dTheta / 2).getRadians();
  Theta -= dTheta;

  // Which wheel is the vertical wheel?
  TrackingWheel* verticalWheel;

  if (leftExists) { verticalWheel = leftWheel; }
  else if (rightExists) { verticalWheel = rightWheel; }
  else
  {
    return;  // TODO: Throw Error
  }

  // Calculate Local Change
  double localDeltaX = this->_calculateChordLength(verticalWheel->getDeltaDistance(),
                                                   verticalWheel->getOffset(), dTheta);
  double localDeltaY = 0;
  if (rearExists)
  {
    localDeltaY = this->_calculateChordLength(rearWheel->getDeltaDistance(),
                                              rearWheel->getOffset(), dTheta);
  }

  // Set Robot Actual Position
  Vector3<double, double, Angle> deltaGlobalPosition(
      localDeltaY * sin(avgTheta) + localDeltaX * cos(avgTheta),
      localDeltaY * cos(avgTheta) + localDeltaX * -sin(avgTheta), dTheta);

  // Update Last Position
  RobotState lastState = currentState;

  // Update Robot State
  currentState.setPosition((*lastState.getPosition()) + deltaGlobalPosition);
  currentState.setVelocity((*currentState.getPosition()) -
                           (*lastState.getPosition()) * dTime / 1000);
  currentState.setAcceleration((*currentState.getVelocity()) -
                               (*lastState.getVelocity()) * dTime / 1000);

  // Update Last Data
  lastTheta = Theta;
}

/**
 * @brief Calculate the Chord Length of a Arc
 *
 * @param length Length of Arc
 * @param offset Tracking Wheel Offset
 * @param theta Measure of Arc
 * @return double Chord Length
 */
double Odometry::_calculateChordLength(double length, double offset, Angle theta)
{
  if (theta.getRawValue() == 0) { return length; }
  else
  {
    double radius = offset + (length / theta.getRadians());
    return 2 * radius * sin(theta.getRadians() / 2);
  }
}

}  // namespace lib2131
