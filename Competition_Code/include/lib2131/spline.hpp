#pragma once

#include <vector>

#include "lib2131/angle.hpp"
#include "lib2131/robot-state.hpp"
#include "lib2131/vector3.hpp"

namespace lib2131
{
class Spline
{
 private:
  float m_resolution;
  std::vector<Vector3<double, double, Angle>> m_TSamples = {};
  RobotState m_P0;
  RobotState m_P1;

 public:
  Spline(RobotState P0, RobotState P1, int resolution)
      : m_P0(P0), m_P1(P1), m_resolution(1 / (float)resolution)
  {
  }

  RobotState calc(float t);
};
}  // namespace lib2131
