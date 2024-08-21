#pragma once
#include "../utilities/Pose.hpp"
#include "../utilities/Velocity.hpp"

namespace lib2131
{
class Chassis
{
 private:
  utilities::Pose currentPosition;
  utilities::Pose targetPostion;

  utilities::Velocity currentVelocity;
  utilities::Velocity targetVelocity;

 public:  // Constructors
  Chassis();

 public:  // Functions
  void move(units::length::inch_t target);
  void moveTo(utilities::Pose target);
  void moveTo(utilities::Point target);

  void turn(units::angle::degree_t degrees);
  void turnTo(utilities::Pose target);
  void turnTo(utilities::Point target);
};

}  // namespace lib2131