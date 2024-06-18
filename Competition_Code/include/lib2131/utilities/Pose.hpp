#pragma once
#include <cmath>

#include "lib2131/utilities/Angle.hpp"

namespace lib2131::utilities
{

struct Pose
{
  double x;
  double y;
  Angle theta;

  double magnitude(const Pose& B)
  {
    return sqrt(pow(B.x - this->x, 2) + pow(B.y - this->y, 2));
  }

  Angle amplitude(const Pose& B)
  {
    return Angle(atan2(B.y - this->y, B.x - this->x), false);
  }

  Pose rotate(Angle& B)
  {
    // Calculate rotated point
    double newX = y * sin(B.getRadians()) + x * cos(B.getRadians());
    double newY = y * cos(B.getRadians()) + x * -sin(B.getRadians());

    // Set new Point
    x = newX;
    y = newY;
    return *this;
  }

  Pose operator+(const Pose& B)
  {
    return {this->x + B.x, this->y + B.y, this->theta + B.theta};
  }
  Pose operator-(const Pose& B)
  {
    return {this->x - B.x, this->y - B.y, this->theta - B.theta};
  }
  Pose operator*(const double B) { return {this->x * B, this->y * B, this->theta * B}; }
  Pose operator/(const double B) { return {this->x / B, this->y / B, this->theta / B}; }

  Pose& operator+=(const Pose& B)
  {
    this->x += B.x;
    this->y += B.y;
    this->theta += B.theta;
    return *this;
  }
  Pose& operator-=(const Pose& B)
  {
    this->x -= B.x;
    this->y -= B.y;
    this->theta -= B.theta;
    return *this;
  }
  Pose& operator*=(const double B)
  {
    this->x *= B;
    this->y *= B;
    this->theta *= B;
    return *this;
  }
  Pose& operator/=(const double B)
  {
    this->x /= B;
    this->y /= B;
    this->theta /= B;
    return *this;
  }
};
}  // namespace lib2131::utilities