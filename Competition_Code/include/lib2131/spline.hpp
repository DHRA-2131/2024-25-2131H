#pragma once

#include <vector>

#include "lib2131/angle.hpp"
#include "lib2131/robot-state.hpp"
#include "lib2131/utilities.hpp"
#include "lib2131/vector3.hpp"

namespace lib2131
{
class Spline
{
 private:
  float m_resolution;
  std::vector<Vector3<double, double, Angle>> m_TSamples;
  RobotState m_P0, m_P1;

 public:
  Spline(RobotState P0, RobotState P1, int resolution)
      : m_P0(P0), m_P1(P1), m_resolution(1 / resolution)
  {
    _SampleSpline();
  }

  RobotState calc(float t)
  {
    Vector3<double, double, Angle> position =
        (m_P0.position * (2 * pow(t, 3) - 3 * pow(t, 2) + 1)) +
        (m_P0.velocity * (pow(t, 3) - 2 * pow(t, 2) + t)) +
        (m_P1.position * (-2 * pow(t, 3) + 3 * pow(t, 2))) +
        (m_P1.velocity * (pow(t, 3) - pow(t, 2)));

    Vector3<double, double, Angle> velocity =
        (m_P0.position * (6 * pow(t, 2) - 6 * t)) +
        (m_P0.velocity * (3 * pow(t, 2) - 4 * t + 1)) +
        (m_P1.position * (-6 * pow(t, 2) + 6 * t)) +
        (m_P1.velocity * (3 * pow(t, 2) - 2 * t));

    Vector3<double, double, Angle> acceleration =
        (m_P0.position * (12 * t - 6)) + (m_P0.velocity * (6 * t - 4)) +
        (m_P1.position * (-12 * t + 6)) + (m_P1.velocity * (6 * t - 2));

    return {position, velocity, acceleration};
  }

  float getNearestT(Vector3<double, double, Angle> position)
  {
    float lastDistance = 0;
    for (size_t i = 0; i <= m_TSamples.size(); i++)
    {
      float dist =
          distance<float>(position.x, position.y, m_TSamples[i].x, m_TSamples[i].y);
      if (dist > lastDistance)
      {
        return (i - 1) * m_resolution;
      }
      else
      {
        lastDistance = dist;
      }
    }
    return 1;
  }

 protected:
  void _SampleSpline()
  {
    for (int i = 0; i <= 1; i += m_resolution)
    {
      m_TSamples.push_back(this->calc(i).position);
    }
  }
};
}  // namespace lib2131
