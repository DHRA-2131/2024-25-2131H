#include "main/Autonomous.hpp"

#include "RobotConfig.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "path-addin/Path.hpp"
#include "systems/Arm.hpp"
#include "systems/Clamp.hpp"
#include "systems/Intake.hpp"

namespace Autonomous
{
using namespace Systems;
class Path : public Pathing::AbstractPath
{
 public:
  lemlib::Pose rootFunction(double t) override
  {
    // X0, Angle0, X1, Angle1
    return this->m_controlPoints[0] + (this->m_controlPoints[0] * -1 + this->m_controlPoints[1]) * t +
           (this->m_controlPoints[0] * -1 - this->m_controlPoints[1] * 2 + this->m_controlPoints[2] * 2 + this->m_controlPoints[3]) *
               std::pow(t, 2) +
           (this->m_controlPoints[0] + this->m_controlPoints[1] - this->m_controlPoints[2] - this->m_controlPoints[3]) * std::pow(t, 3);
  };
  lemlib::Pose firstDerivative(double t) override
  {
    return this->m_controlPoints[0] + (this->m_controlPoints[0] * -1 + this->m_controlPoints[1]) +
           (this->m_controlPoints[0] * -1 - this->m_controlPoints[1] * 2 + this->m_controlPoints[2] * 2 + this->m_controlPoints[3]) * t *
               2 +
           (this->m_controlPoints[0] + this->m_controlPoints[1] - this->m_controlPoints[2] - this->m_controlPoints[3]) * std::pow(t, 2) * 3;
  };
  lemlib::Pose secondDerivative(double t) override
  {
    return (this->m_controlPoints[0] * -1 - this->m_controlPoints[1] * 2 + this->m_controlPoints[2] * 2 + this->m_controlPoints[3]) * 2 +
           (this->m_controlPoints[0] + this->m_controlPoints[1] - this->m_controlPoints[2] - this->m_controlPoints[3]) * t * 6;
  };
} mySpline;

void lowStake(bool isRedTeam)
{
  if (isRedTeam) { chassis.setPose(48, 15.5, -180); }
  else { chassis.setPose(81.5, 18.0, -180); }
  chassis.turnToPoint(72, 12, 1000);
  chassis.moveToPoint(72, 12, 1000);
  chassis.swingToPoint(72, 0, lemlib::DriveSide::LEFT, 1000);
  Arm::setPosition(2);  // Extend Arm
}
void highStake(bool isRedTeam)
{
  Intake::disableAutoSort();
  if (isRedTeam) { chassis.setPose(48, 94, 0); }
  else { chassis.setPose(48, 20, -180); }
  Clamp::enableAutoClamp();
  chassis.moveToPoint(48, 54, 1000, {false, 80}, false);
  pros::delay(100);
  Intake::motor.move_voltage(12000);
  chassis.moveToPoint(24, 48, 1000, {});
  chassis.turnToHeading(90, 800, {}, false);
  Clamp::disableAutoClamp();
  chassis.moveToPoint(72, 24, 1500, {true, 80}, false);
  Clamp::enableAutoClamp();
  chassis.moveToPoint(100, 52, 1000, {false, 80});
  chassis.turnToPoint(120, 47, 800);
  chassis.moveToPoint(120, 47, 1000);
}
void skills(bool isRedTeam) {}
void debug(bool isRedTeam)
{
  chassis.setPose({0, 0, 0});
  mySpline.setControlPoints({{0, 0, 0}, {0, 10, 0}, {10, 10, 90}, {0, 10, -90}});

  chassis.follow(mySpline, 100, 100, {true, 127});
}
}  // namespace Autonomous