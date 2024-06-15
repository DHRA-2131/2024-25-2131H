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
    : m_position(0, 0, Angle()), m_velocity(0, 0, Angle()), m_acceleration(0, 0, Angle())
{
}

/**
 * @brief Construct a new robot State object
 *
 * @param Position {x, y, θ} as Vector3 Object
 * @param Velocity {vx, vy, w} as Vector3 Object
 * @param Acceleration {ax, ay, α} as Vector3 Object
 */
RobotState::RobotState(Vector3<double, double, Angle> Position,
                       Vector3<double, double, Angle> Velocity,
                       Vector3<double, double, Angle> Acceleration)
    : m_position(Position), m_velocity(Velocity), m_acceleration(Acceleration)
{
}

/**
 * @brief Get the Position of RobotState
 *
 * @return Vector3<double, double, Angle> Position
 */
Vector3<double, double, Angle>* RobotState::getPosition() { return &m_position; }
/**
 * @brief Get the Velocity at RobotState
 *
 * @return Vector3<double, double, Angle> Velocity
 */
Vector3<double, double, Angle>* RobotState::getVelocity() { return &m_velocity; }

/**
 * @brief Get the Acceleration at RobotState
 *
 * @return Vector3<double, double, Angle> Acceleration
 */
Vector3<double, double, Angle>* RobotState::getAcceleration() { return &m_acceleration; }
/**
 * @brief Set the Position of RobotState
 *
 * @param newPosition
 */
void RobotState::setPosition(Vector3<double, double, Angle> newPosition)
{
  m_position = newPosition;
}
/**
 * @brief Set the Velocity of RobotState
 *
 * @param newVelocity
 */
void RobotState::setVelocity(Vector3<double, double, Angle> newVelocity)
{
  m_velocity = newVelocity;
}
/**
 * @brief Set the Acceleration of RobotState
 *
 * @param newAcceleration
 */
void RobotState::setAcceleration(Vector3<double, double, Angle> newAcceleration)
{
  m_acceleration = newAcceleration;
}

/**
 * @brief Addition Operator
 *
 * @param B Added RobotState
 * @return RobotState Sum of RobotStates
 */
RobotState RobotState::operator+(RobotState B)
{
  return {this->m_position + (*B.getPosition()), this->m_velocity + (*B.getVelocity()),
          this->m_acceleration + (*B.getAcceleration())};
}
/**
 * @brief Subtraction Operator
 *
 * @param B Subtracting RobotState
 * @return RobotState Difference of RobotStates
 */
RobotState RobotState::operator-(RobotState B)
{
  return {this->m_position - (*B.getPosition()), this->m_velocity - (*B.getVelocity()),
          this->m_acceleration - (*B.getAcceleration())};
}
/**
 * @brief Multiplication Operator
 *
 * @param B Scalar
 * @return RobotState Scaled RobotState
 */
RobotState RobotState::operator*(double B)
{
  return {this->m_position * B, this->m_velocity * B, this->m_acceleration * B};
}
/**
 * @brief Division Operator
 *
 * @param B Scalar
 * @return RobotState Scaled RobotState
 */
RobotState RobotState::operator/(double B)
{
  return {this->m_position / B, this->m_velocity / B, this->m_acceleration / B};
}

/**
 * @brief Addition Assignment
 * @note Adds to Parent RobotState
 * @param B Added RobotState
 */
void RobotState::operator+=(RobotState B)
{
  this->m_position += (*B.getPosition());
  this->m_velocity += (*B.getVelocity());
  this->m_acceleration += (*B.getAcceleration());
}

/**
 * @brief Subtraction Assignment
 * @note Subtracts from Parent RobotState
 * @param B Subtracting RobotState
 */
void RobotState::operator-=(RobotState B)
{
  this->m_position -= (*B.getPosition());
  this->m_velocity -= (*B.getVelocity());
  this->m_acceleration -= (*B.getAcceleration());
}
/**
 * @brief Multiplication Assignment
 * @note Scales Parent RobotState
 * @param B Scalar RobotState
 */
void RobotState::operator*=(double B)
{
  this->m_position *= B;
  this->m_velocity *= B;
  this->m_acceleration *= B;
}
/**
 * @brief Division Assignment
 * @note Scales Parent RobotState
 * @param B Scalar RobotState
 */
void RobotState::operator/=(double B)
{
  this->m_position /= B;
  this->m_velocity /= B;
  this->m_acceleration /= B;
}
/**
 * @brief Print RobotState to Ostream
 *
 * @param os Output Stream
 * @param B RobotState
 * @return std::ostream& os
 */
std::ostream& operator<<(std::ostream& os, const RobotState& B)
{
  os << "{" << B.m_position << ", " << B.m_velocity << ", " << B.m_acceleration << "}";
  return os;
}
}  // namespace lib2131