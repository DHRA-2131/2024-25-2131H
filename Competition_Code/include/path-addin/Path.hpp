#pragma once
#include <cmath>
#include <vector>

#include "lemlib/pose.hpp"

namespace Pathing
{
class AbstractPath
{
 public:  // Variables
  std::vector<lemlib::Pose> m_controlPoints;

 public:  // Overrides
  virtual lemlib::Pose rootFunction(double t) = 0;
  virtual lemlib::Pose firstDerivative(double t) = 0;
  virtual lemlib::Pose secondDerivative(double t) = 0;

 public:  // Functions
  void setControlPoints(std::vector<lemlib::Pose> newControlPoints) { m_controlPoints = newControlPoints; }
  double getLength(double samples = 100)
  {
    double length(0.0);
    for (double i = 1; i < samples; i++) { length += rootFunction(i / samples).distance(rootFunction((i - 1.0) / samples)); }
    return length;
  }
  virtual double curvature(double t)
  {
    lemlib::Pose first = firstDerivative(t);
    lemlib::Pose second = secondDerivative(t);

    return ((first.x * second.y - first.y * second.x) / std::pow(std::pow(first.x, 2) + std::pow(first.y, 2), 3 / 2));
  }

  virtual double distanceToRobot(double t, lemlib::Pose Pos)
  {
    lemlib::Pose root = rootFunction(t);

    return std::sqrt(std::pow(root.x - Pos.x, 2) + std::pow(root.y - Pos.y, 2));
  }

  virtual double findClosestT(lemlib::Pose pos, int depth)
  {
    double t0 = 0.0;
    double t1 = 1.0;
    double t2;

    for (int i = 0; i < depth; i++)
    {
      double ft1 = this->distanceToRobot(t1, pos);
      double ft0 = this->distanceToRobot(t0, pos);

      double dt = t1 - t0;
      double dy = ft1 - ft0;

      if (dt == 0.0 or dy == 0.0) return t1;

      t2 = t1 - ft1 * dt / dy;
      t0 = t1;
      t1 = t2;
    }
    return std::min(std::max(t2, 0.0), 1.0);
  }
};
}  // namespace Pathing