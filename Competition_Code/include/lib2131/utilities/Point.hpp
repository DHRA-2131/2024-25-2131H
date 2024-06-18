#pragma once
#include <cmath>

#include "lib2131/utilities/Angle.hpp"

namespace lib2131::utilities
{

struct Point
{
  double x, y;

  double magnitude(const Point& B)
  {
    return sqrt(pow(B.x - this->x, 2) + pow(B.y - this->y, 2));
  }

  Angle amplitude(const Point& B)
  {
    return Angle(atan2(B.y - this->y, B.x - this->x), false);
  }

  void rotate(Angle& B)
  {
    // Calculate rotated point
    double newX = y * sin(B.getRadians()) + x * cos(B.getRadians());
    double newY = y * cos(B.getRadians()) + x * -sin(B.getRadians());

    // Set new Point
    x = newX;
    y = newY;
  }

  const Point operator+(const Point& B) { return {this->x + B.x, this->y + B.y}; }
  const Point operator-(const Point& B) { return {this->x - B.x, this->y - B.y}; }
  const Point operator*(const double B) { return {this->x * B, this->y * B}; }
  const Point operator/(const double B) { return {this->x / B, this->y / B}; }

  Point& operator+=(const Point& B)
  {
    this->x += B.x;
    this->y += B.y;
    return *this;
  }
  Point& operator-=(const Point& B)
  {
    this->x -= B.x;
    this->y -= B.y;
    return *this;
  }
  Point& operator*=(const double B)
  {
    this->x *= B;
    this->y *= B;
    return *this;
  }
  Point& operator/=(const double B)
  {
    this->x /= B;
    this->y /= B;
    return *this;
  }
};
}  // namespace lib2131::utilities