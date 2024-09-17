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

  virtual double secondDistanceToPointEquation(double t, lemlib::Pose Pos, double dt)
  {
    double f = firstDistanceToPointEquation(t, Pos);
    double df = firstDistanceToPointEquation(t + dt, Pos);

    return (f - df) / dt;
  }

  virtual double findClosestT(double t, lemlib::Pose Pos, int depth,
                              double dt = 0.00000000001)
  {
    double f = firstDistanceToPointEquation(t, Pos);
    double df = firstDistanceToPointEquation(t + dt, Pos);

    double guess = 0.5;
    for (int i = 0; i < depth; i++) { guess = guess - f / df; }

    return guess;
  }
};
}  // namespace Path