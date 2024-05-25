/**
 * @file robot-state.hpp
 * @author Andrew Hilton (2131H)
 * @brief Robot State Source Code
 * @version 0.1
 * @date 2024-05-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lib2131/robot-state.hpp"
namespace lib2131
{

/**
 * @brief Construct a empty robot State object
 *
 */
robotState::robotState()
    : position(0, 0, angle()), velocity(0, 0, angle()), acceleration(0, 0, angle())
{
}

/**
 * @brief Construct a new robot State object
 *
 * @param Position X, Y, Theta as Vector3 Object
 * @param Velocity X', Y', Theta' as Vector3 Object
 * @param Acceleration X'', Y'', Theta'' as Vector3 Object
 */
robotState::robotState(Vector3<double, double, angle> Position,
                       Vector3<double, double, angle> Velocity,
                       Vector3<double, double, angle> Acceleration)
    : position(Position), velocity(Velocity), acceleration(Acceleration)
{
}

}  // namespace lib2131