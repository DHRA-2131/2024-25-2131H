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
    this->waitUntilDone();
    lemlib::Pose pose = this->getPose(true);

    dist < 0 ? p.forwards = false : p.forwards = true;

    this->moveToPoint(
        pose.x + std::sin(pose.theta) * dist,
        pose.y + std::cos(pose.theta) * dist,
        timeout,
        p,
        async);
  }

  void movePolar(
      double dist,
      double angle,
      int timeout,
      lemlib::MoveToPointParams p = {},
      bool radians = false,
      bool async = true)
  {
    this->waitUntilDone();
    lemlib::Pose pose = this->getPose(true);

    dist < 0 ? p.forwards = false : p.forwards = true;
    angle = radians ? angle : angle / 180.0 * M_PI;

    this->moveToPoint(
        pose.x + std::sin(angle) * dist, pose.y + std::cos(angle) * dist, timeout, p, async);
  }

  void attemptReckonToGoal(lemlib::Pose goalPose, Clamp *c, int timeout, float goalSize = 10.0)
  {
    auto start = pros::millis();
    while (!c->isGoal() && pros::millis() < start + timeout) { pros::delay(10); }
    if (c->isGoal())
    {
      auto pose = this->getPose(true);

      this->setPose(
          goalPose.x + (goalSize)*sin(pose.theta) +
              sin(pose.theta) * (14 / 2.0 - c->getGoalIndent()),  //
          goalPose.y + (goalSize)*cos(-pose.theta) +
              cos(-pose.theta) * (14 / 2.0 - c->getGoalIndent()),
          pose.theta,
          true);

      auto pose2 = this->getPose();

      std::cout << "RECKONED TO GOAL" << std::endl;

      this->cancelAllMotions();
    }
    else
    {
      // NO GOAL DETECTED
    }
  }
  void shimmy(int timeout, int delay = 100)
  {
    auto start = pros::millis();
    while (pros::millis() < start + timeout)
    {
      this->drivetrain.leftMotors->move_voltage(-12000);
      this->drivetrain.rightMotors->move_voltage(12000);
      pros::delay(delay);
      this->drivetrain.leftMotors->move_voltage(12000);
      this->drivetrain.rightMotors->move_voltage(-12000);
      pros::delay(delay);
    }
  }

 protected:
};