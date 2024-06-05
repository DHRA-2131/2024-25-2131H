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
 * @brief Construct a empty RobotState object
 *
 */
RobotState::RobotState()
    : position(0, 0, Angle()), velocity(0, 0, Angle()), acceleration(0, 0, Angle())
{
}

/**
 * @brief Construct a new RobotState object
 *
 * @param Position X, Y, Theta as Vector3 Object
 * @param Velocity X', Y', Theta' as Vector3 Object
 * @param Acceleration X'', Y'', Theta'' as Vector3 Object
 */
RobotState::RobotState(Vector3<double, double, Angle> Position,
                       Vector3<double, double, Angle> Velocity,
                       Vector3<double, double, Angle> Acceleration)
    : position(Position), velocity(Velocity), acceleration(Acceleration)
{
}

}  // namespace lib2131