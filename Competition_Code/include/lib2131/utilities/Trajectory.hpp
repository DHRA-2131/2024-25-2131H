#pragma once

#include "Point.hpp"
#include "Pose.hpp"
#include "Units.h"

namespace lib2131::utilities
{
using namespace units::literals;
class Trajectory
{
 private:
  utilities::Pose currentPose;  // t = 0
  utilities::Pose targetPose;   // t = 1

  utilities::Point currentHeading;
  utilities::Point targetHeading;

  // Affects the curvature and heading of the spline
  units::length::inch_t currentPoseStrength;
  units::length::inch_t targetPoseStrength;

  units::length::inch_t pathLength;
  int approximationDepth;

 public:  // Constructors
  Trajectory(utilities::Pose currentPose, utilities::Pose targetPose,
             units::length::inch_t currentPoseStrength,
             units::length::inch_t targetPoseStrength, int approximationDepth = 30)
      : currentPose(currentPose),
        targetPose(targetPose),
        currentPoseStrength(currentPoseStrength),
        targetPoseStrength(targetPoseStrength),
        currentHeading({units::math::cos(currentPose.theta) * currentPoseStrength +
                            currentPose.pos.x,
                        units::math::sin(currentPose.theta) * currentPoseStrength +
                            currentPose.pos.y}),
        targetHeading(
            {units::math::cos(currentPose.theta) * targetPoseStrength + targetPose.pos.x,
             units::math::sin(targetPose.theta) * targetPoseStrength + targetPose.pos.y}),
        approximationDepth(approximationDepth)
  {
    for (double i = 0; i < approximationDepth; i++)
    {
      pathLength += this->getPosition(i / approximationDepth)
                        .magnitude(this->getPosition((i + 1) / approximationDepth));
    }
  }

  units::length::inch_t getPathLength() { return pathLength; }

  utilities::Point getPosition(double t)
  {
    return currentPose.pos + (currentPose.pos * -1 + currentHeading) * t +
           (currentPose.pos * -1 - currentHeading * 2 + targetPose.pos * 2 +
            targetHeading) *
               t * t +
           (currentPose.pos + currentHeading - targetPose.pos - targetHeading) * t * t *
               t;
  }

  double getCurvature(double t)
  {
    auto velocity = _XYVelocity(t);
    auto acceleration = _XYAcceleration(t);

    return units::math::abs((velocity.x * acceleration.y - velocity.y * acceleration.x) /
                            units::math::pow<2>(velocity.x)) /
           units::math::pow<3 / 2>(1 + units::math::pow<2>(velocity.y / velocity.x));
  }

  double solveFromDistance(units::length::inch_t distance, double startT)
  {
    double i = int(startT * 100);
    units::length::inch_t summedDistance;
    while (i < 100 && summedDistance < distance)
    {
      summedDistance +=
          this->getPosition(i / 100).magnitude(this->getPosition((i + 1) / 100));
      i += 1;
    }
    return i / 100;
  }

 private:
  utilities::Point _XYVelocity(double t)
  {
    return (currentPose.pos * -1 + currentHeading) +
           (currentPose.pos * -1 - currentHeading * 2 + targetPose.pos * 2 +
            targetHeading) *
               2 * t +
           (currentPose.pos + currentHeading - targetPose.pos - targetHeading) * 3 * t *
               t;
  }
  utilities::Point _XYAcceleration(double t)
  {
    return (currentPose.pos * -1 - currentHeading * 2 + targetPose.pos * 2 +
            targetHeading) *
               2 +
           (currentPose.pos + currentHeading - targetPose.pos - targetHeading) * 6 * t;
  }
};

}  // namespace lib2131::utilities