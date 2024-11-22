/**
 * @file pose.hpp
 * @author Andrew Hilton (2131H)
 * @brief Class Declaration for the Pose struct. Basic Pose containing a (X, Y)
 * Position and a Yaw Rotation (Theta)
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <cmath>
namespace Utilities
{
class Pose
{
 private:          // === Member Variables === //
  double m_theta;  // Yaw Rotation in Radians
                   //! USES RADIANS INTERNALLY

 public:     //  === Public Variables === //
  double x;  // X Position
  double y;  // Y Position

 public:  // === Constructors === //
  /**
   * @brief Construct a new Pose
   *
   * @param x X Value
   * @param y Y Value
   * @param theta Yaw Rotation in Degrees
   */
  Pose(double x, double y, double theta, bool radians = false)
      : x(x), y(y), m_theta(radians ? theta : theta * M_PI / 180.0)  // Converts to radians
  {
  }

 public:  // === Getter / Setters === /
  /**
   * @brief Get the Theta of the Pose
   *
   * @param radians Return in radians?
   * @return double
   */
  double getTheta(bool radians = false) const { return radians ? m_theta : m_theta * 180.0 / M_PI; }

  /**
   * @brief Set the Theta of the Pose
   *
   * @param newTheta New measure for theta
   * @param radians Is measure in radians
   */
  void setTheta(double newTheta, bool radians = false)
  {
    m_theta = radians ? newTheta : newTheta * M_PI / 180.0;
  }

 public:  // === Functions === /
  /**
   * @brief Calculates distance from one Pose to another
   *
   * @param B Second Pose
   * @return double
   */
  double distance(const Pose B) const { return std::hypot(this->x - B.x, this->y - B.y); }

  /**
   * @brief Calculates the angle from one pose to another
   *
   * @param B target pose
   * @param radians Is output in radians? False by default.
   * @return double Angle in degrees or radians
   */
  double angle(const Pose B, bool radians = false) const
  {
    double rawAngle = std::atan2(B.y - this->y, B.x - this->x);
    return radians ? rawAngle : rawAngle * 180 / M_PI;
  }
  /**
   * @brief returns Pose rotated by angle value
   *
   * @param angle measure of angle
   * @param radians Is angle in Radians?
   * @return Pose
   */
  Pose rotate(float angle, bool radians = false) const
  {
    if (!radians) { angle = angle * M_PI / 180; }  // Force Radians
    return {this->x * std::cos(angle) - this->y * std::sin(angle),
            this->x * std::sin(angle) + this->y * std::cos(angle), this->m_theta};
  }

 public:  // === Operator Overloads === /
  /**
   * @brief Sum of Two Poses
   * @note Theta Values aren't Summed
   * @param B Added Pose
   * @return Pose
   */
  Pose operator+(const Pose &B) const { return {this->x + B.x, this->y + B.y, this->m_theta}; }
  /**
   * @brief Difference of Two Poses
   * @note Theta Values aren't subtracted
   * @param B Subtracting Pose
   * @return Pose
   */
  Pose operator-(const Pose &B) const { return {this->x - B.x, this->y - B.y, this->m_theta}; }

  /**
   * @brief Scales Pose by value B
   * @note Doesn't Affect Theta
   * @param B Scalar value
   * @return Pose
   */
  Pose operator*(const double &B) const { return {this->x * B, this->y * B, this->m_theta}; }
  /**
   * @brief Scales Pose by value B
   * @note Doesn't Affect Theta
   * @param B Scalar value
   * @return Pose
   */
  Pose operator/(const double &B) const { return {this->x / B, this->y / B, this->m_theta}; }
};
}  // namespace Utilities