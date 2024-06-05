/**
 * @file vector3.hpp
 * @author Andrew Hilton (2131H)
 * @brief File for Vector3 Struct.
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include <ostream>

namespace lib2131
{

/**
 * @brief Stores three values
 *
 * @tparam X_Type
 * @tparam Y_Type
 * @tparam Z_Type
 */
// TODO: Make sure this throws an error when types don't match
template <typename X_Type, typename Y_Type, typename Z_Type>
struct Vector3
{
  X_Type x;
  Y_Type y;
  Z_Type z;

  /**
   * @brief Construct a empty Vector 3 object
   *
   */
  Vector3() {}

  /**
   * @brief Construct a new Vector 3 object
   *
   * @param X X-Value
   * @param Y Y-Value
   * @param Z Z-Value
   */
  Vector3(X_Type X, Y_Type Y, Z_Type Z) : x(X), y(Y), z(Z) {}

  /**
   * @brief Add to Vector
   *
   * @param B Added Vector
   */
  void operator+=(const Vector3 &B)
  {
    this->x += B.x;
    this->y += B.y;
    this->z += B.z;
  }

  /**
   * @brief Subtract Vector
   *
   * @param B Subtracting Vector
   */
  void operator-=(const Vector3 &B)
  {
    this->x -= B.x;
    this->y -= B.y;
    this->z -= B.z;
  }

  /**
   * @brief Scale vector
   *
   * @param B Scalar
   */
  void operator*=(const double B)
  {
    this->x *= B;
    this->y *= B;
    this->z *= B;
  }

  /**
   * @brief Scale Vector
   *
   * @param B Scalar
   */
  void operator/=(const double B)
  {
    this->x /= B;
    this->y /= B;
    this->z /= B;
  }

  /**
   * @brief Add two Vector3s
   *
   * @param B Added Vector
   * @return Vector3<X_Type, Y_Type, Z_Type> Sum of Vectors
   */
  Vector3<X_Type, Y_Type, Z_Type> operator+(Vector3 B)
  {
    return Vector3<X_Type, Y_Type, Z_Type>(this->x + B.x, this->y + B.y, this->z + B.z);
  }

  /**
   * @brief Subtract two Vector3s
   *
   * @param B Subtracting Vector
   * @return Vector3<X_Type, Y_Type, Z_Type> Difference of Vectors
   */
  Vector3<X_Type, Y_Type, Z_Type> operator-(Vector3 B)
  {
    return Vector3<X_Type, Y_Type, Z_Type>(this->x - B.x, this->y - B.y, this->z - B.z);
  }

  /**
   * @brief Scale Vector
   *
   * @param B Scalar
   * @return Vector3<X_Type, Y_Type, Z_Type> Scaled Vector
   */
  Vector3<X_Type, Y_Type, Z_Type> operator*(double B)
  {
    return Vector3<X_Type, Y_Type, Z_Type>(this->x * B, this->y * B, this->z * B);
  }

  /**
   * @brief Scale Vector
   *
   * @param B Scalar
   * @return Vector3<X_Type, Y_Type, Z_Type> Scaled Vector
   */
  Vector3<X_Type, Y_Type, Z_Type> operator/(double B)
  {
    return Vector3<X_Type, Y_Type, Z_Type>(this->x / B, this->y / B, this->z / B);
  }

  friend std::ostream &operator<<(std::ostream &os, const Vector3 &B)
  {
    os << "{" << B.x << ", " << B.y << ", " << B.z << "}";
    return os;
  }
};
}  // namespace lib2131