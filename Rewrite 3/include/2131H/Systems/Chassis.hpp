#pragma once

#include "cmath"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

class Chassis : public lemlib::Chassis
{
 public:
  void moveLinear(double dist, int timeout, lemlib::MoveToPointParams p = {}, bool async = true);

  void movePolar(
      double dist,
      double angle,
      int timeout,
      lemlib::MoveToPointParams p = {},
      bool radians = false,
      bool async = true);

  void movePolarAbsolute(
      lemlib::Pose pose,
      double dist,
      double angle,
      int timeout,
      lemlib::MoveToPointParams p,
      bool radians,
      bool async);

  void shimmy(int timeout, int delay = 100);

  void brake();
};