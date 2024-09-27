#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "path-addin/MotionProfile.hpp"
#include "path-addin/Path.hpp"

namespace Pathing
{
class Chassis : public lemlib::Chassis
{
 public:
  Chassis(lemlib::Drivetrain drivetrain, lemlib::ControllerSettings linearSettings, lemlib::ControllerSettings angularSettings,
          lemlib::OdomSensors sensors, lemlib::DriveCurve* throttleCurve = &lemlib::defaultDriveCurve,
          lemlib::DriveCurve* steerCurve = &lemlib::defaultDriveCurve)
      : lemlib::Chassis(drivetrain, linearSettings, angularSettings, sensors, throttleCurve, steerCurve)
  {
  }

  void follow(AbstractPath& path, double maxAccel, double maxDecel, lemlib::MoveToPointParams params = {}, double initalVelocity = 0,
              double endVelocity = 0)
  {
    MotionProfile MP(path.getLength(), ((this->drivetrain.rpm * M_PI * this->drivetrain.wheelDiameter) / 60.0) * (params.maxSpeed / 127.0),
                     maxAccel, maxAccel, initalVelocity, endVelocity);
    double time = 0;
    double t = 0;
    while (t < 1)
    {
      t = path.findClosestT(this->getPose(), 30);

      double linearVel = MP.getVelocity(time);
      double angularVel = linearVel * path.curvature(t);

      double left = (linearVel - angularVel) / (M_PI * this->drivetrain.wheelDiameter);   // Motor velocity in ptc
      double right = (linearVel + angularVel) / (M_PI * this->drivetrain.wheelDiameter);  // Motor velocity in ptc

      this->drivetrain.leftMotors->move_voltage(left * 12000);
      this->drivetrain.rightMotors->move_voltage(right * 12000);

      pros::delay(10);
      time += 10;
    }
  }
};
}  // namespace Pathing