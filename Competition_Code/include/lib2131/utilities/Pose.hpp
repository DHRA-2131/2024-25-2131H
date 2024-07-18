#pragma once
#include <cmath>

#include "lib2131/utilities/Point.hpp"
#include "lib2131/utilities/Units.h"

namespace lib2131::utilities
{

struct Pose
{
  using angle_t = units::angle::radian_t;
  Point pos;
  angle_t theta;

  Pose operator+(const Pose& B) { return {this->pos + B.pos, this->theta + B.theta}; }
  Pose operator-(const Pose& B) { return {this->pos - B.pos, this->theta - B.theta}; }
  Pose operator*(const double B) { return {this->pos * B, this->theta * B}; }
  Pose operator/(const double B) { return {this->pos / B, this->theta / B}; }

  Pose& operator+=(const Pose& B)
  {
    this->pos += B.pos;
    this->theta += B.theta;
    return *this;
  }
  Pose& operator-=(const Pose& B)
  {
    this->pos -= B.pos;
    this->theta -= B.theta;
    return *this;
  }
  Pose& operator*=(const double B)
  {
    this->pos *= B;
    this->theta *= B;
    return *this;
  }
  Pose& operator/=(const double B)
  {
    this->pos /= B;
    this->theta /= B;
    return *this;
  }
};
}  // namespace lib2131::utilities