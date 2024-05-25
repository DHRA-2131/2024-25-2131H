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
#include "lib2131/vector3.hpp"
#include "utilities.hpp"

namespace lib2131
{

/**
 * @brief robotState Struct. Defines Robot's Speed, Velocity, and Acceleration.
 *
 */
struct robotState
{
  Vector3<double, double, angle> position;
  Vector3<double, double, angle> velocity;
  Vector3<double, double, angle> acceleration;

  /**
   * @brief Construct a empty robot State object
   *
   */
  robotState();

  /**
   * @brief Construct a new robot State object
   *
   * @param Position X, Y, Theta as Vector3 Object
   * @param Velocity X', Y', Theta' as Vector3 Object
   * @param Acceleration X'', Y'', Theta'' as Vector3 Object
   */
  robotState(Vector3<double, double, angle> Position,
             Vector3<double, double, angle> Velocity,
             Vector3<double, double, angle> Acceleration);
};
}  // namespace lib2131