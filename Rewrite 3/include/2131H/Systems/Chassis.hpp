#pragma once

#include "2131H/Systems/Clamp.hpp"
#include "cmath"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

class Chassis : public lemlib::Chassis
{
 private:
 public:
  void moveLinear(double dist, int timeout, lemlib::MoveToPointParams p = {}, bool async = true)
  {
    lemlib::Pose pose = this->getPose();
    this->moveToPoint(
        pose.x + std::sin(pose.theta) * dist,
        pose.y + std::cos(pose.theta) * dist,
        timeout,
        p,
        async);
  }

  void attemptReckonToGoal(lemlib::Pose goalPose, Clamp *c, int timeout, float goalSize = 10.0)
  {
    auto start = pros::millis();
    while (!c->isGoal() && pros::millis() < start + timeout) { pros::delay(10); }
    if (c->isGoal())
    {
      auto pose = this->getPose(true);

      this->setPose(
          pose.x + (goalSize - c->getGoalIndent()) * sin(pose.theta) +
              sin(pose.theta) * this->drivetrain.trackWidth / 2.0,  //
          pose.y + (goalSize - c->getGoalIndent()) * cos(pose.theta) +
              cos(pose.theta) * this->drivetrain.trackWidth / 2.0,
          pose.theta,
          true);
    }
    else
    {
      // NO GOAL DETECTED
    }
  }

 protected:
};