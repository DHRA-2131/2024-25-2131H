/**
 * @file robot-state.hpp
 * @author Andrew Hilton (2131H)
 * @brief File for robotState struct that defines the robots: Position, Speed, and
 * Acceleration
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "lib2131/angle.hpp"
#include "lib2131/vector3.hpp"

namespace lib2131
{

/**
 * @brief robotState Struct. Defines Robot's Speed, Velocity, and Acceleration.
 *
 */
struct RobotState
{
  Vector3<double, double, Angle> position;
  Vector3<double, double, Angle> velocity;
  Vector3<double, double, Angle> acceleration;

  /**
   * @brief Construct a empty robot State object
   *
   */
  RobotState();

  /**
   * @brief Construct a new robot State object
   *
   * @param Position X, Y, Theta as Vector3 Object
   * @param Velocity X', Y', Theta' as Vector3 Object
   * @param Acceleration X'', Y'', Theta'' as Vector3 Object
   */
  RobotState(Vector3<double, double, Angle> Position,
             Vector3<double, double, Angle> Velocity,
             Vector3<double, double, Angle> Acceleration);
};
}  // namespace lib2131