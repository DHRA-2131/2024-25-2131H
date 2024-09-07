#pragma once
#include <cmath>

#include "lemlib/pose.hpp"

namespace Path
{
class AbstractPath
{
 private:  // Variables
 public:   // Constructors
  AbstractPath();
  ~AbstractPath();

 public:  // Functions
  virtual lemlib::Pose rootFunction(double t) = 0;
  virtual lemlib::Pose firstDerivative(double t) = 0;
  virtual lemlib::Pose secondDerivative(double t) = 0;

 protected:  // Functions
  virtual double curvature(double t)
  {
    lemlib::Pose first = firstDerivative(t);
    lemlib::Pose second = secondDerivative(t);

    return ((first.x * second.y - first.y * second.x) /
            std::pow(std::pow(first.x, 2) + std::pow(first.y, 2), 3 / 2));
  }

  virtual double rootDistanceToPointEquation(double t, lemlib::Pose Pos)
  {
    lemlib::Pose root = rootFunction(t);

    return std::sqrt(std::pow(root.x - Pos.x, 2) + std::pow(root.y - Pos.y, 2));
  }

  virtual double firstDistanceToPointEquation(double t, lemlib::Pose Pos)
  {
    lemlib::Pose root = rootFunction(t);
    lemlib::Pose first = firstDerivative(t);

    return ((root.x - Pos.x) * first.x + (root.y - Pos.y) * first.y) /
           std::sqrt(std::pow(root.x - Pos.x, 2) + std::pow(root.y - Pos.y, 2));
  }

  virtual double secondDistanceToPointEquation(double t, lemlib::Pose Pos)
  {
    lemlib::Pose root = rootFunction(t);
    lemlib::Pose first = firstDerivative(t);
    lemlib::Pose second = secondDerivative(t);

    return ((root.x - Pos.x) * second.x + (root.y - Pos.y) * second.y +
            std::pow(first.x, 2) + std::pow(first.y, 2)) /
               std::sqrt(std::pow(root.x - Pos.x, 2) + std::pow(root.y - Pos.y, 2)) -
           ((((root.x - Pos.x) * first.x + (root.y - Pos.y) * first.y)) *
            (2 * (root.x - Pos.x) * first.x + 2 * (root.y - Pos.y) * first.y)) /
               std::pow(2 * std::pow(root.x - Pos.x, 2) + std::pow(root.y - Pos.y, 2),
                        3 / 2);
  }

  virtual double findClosestT(double t, lemlib::Pose, int depth)
  {
    lemlib::Pose root = rootFunction(t);
    lemlib::Pose first = firstDerivative(t);
    return 0;
  }
};
}  // namespace Path