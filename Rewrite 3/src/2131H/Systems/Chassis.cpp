#include "2131H/Systems/Chassis.hpp" 


void Chassis::moveLinear(double dist, int timeout, lemlib::MoveToPointParams p, bool async)
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

void Chassis::movePolar(
    double dist,
    double angle,
    int timeout,
    lemlib::MoveToPointParams p,
    bool radians,
    bool async)
{
this->waitUntilDone();
lemlib::Pose pose = this->getPose(true);

dist < 0 ? p.forwards = false : p.forwards = true;
angle = radians ? angle : angle / 180.0 * M_PI;

this->moveToPoint(
    pose.x + std::sin(angle) * dist, pose.y + std::cos(angle) * dist, timeout, p, async);
}

void Chassis::movePolarAbsolute(
    lemlib::Pose pose,
    double dist,
    double angle,
    int timeout,
    lemlib::MoveToPointParams p,
    bool radians,
    bool async)
{
this->waitUntilDone();

dist < 0 ? p.forwards = false : p.forwards = true;
angle = radians ? angle : angle / 180.0 * M_PI;

this->moveToPoint(
    pose.x + std::sin(angle) * dist, pose.y + std::cos(angle) * dist, timeout, p, async);
}
  
void Chassis::shimmy(int timeout, int delay)
{
this->cancelAllMotions();
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

this->drivetrain.leftMotors->move_voltage(0);
this->drivetrain.rightMotors->move_voltage(0);
}