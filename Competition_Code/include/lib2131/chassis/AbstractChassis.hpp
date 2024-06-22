#pragma once

#include <cstdint>
#include <memory>

#include "lib2131/controller/AbstractController.hpp"
#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/utilities/Point.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

namespace lib2131::chassis
{

struct ChassisParameters
{
  const double wheelDiameter;          // Inches
  const double wheelRpm;               // RPM
  const double maxLinearVelocity;      // In / S
  const double maxLinearAcceleration;  // In / S^2
  const double maxLinearDeceleration;  // In / S^2

  const double maxAngularVelocity;      // Rad / S
  const double maxAngularAcceleration;  // Rad / S^2
  const double maxAngularDeceleration;  // Rad / S^2
};

class AbstractChassis
{
 protected:  // Variables
  std::shared_ptr<controller::AbstractController> m_controller;

  // Motor Groups
  pros::v5::MotorGroup m_leftDrive;
  pros::v5::MotorGroup m_rightDrive;
  pros::motor_brake_mode_e m_brakeType;

 public:  // Variables
  std::shared_ptr<odometry::AbstractOdometry> m_odometry;

 public:  // Constructors
  AbstractChassis(const std::initializer_list<std::int8_t> leftPorts,
                  const std::initializer_list<std::int8_t> rightPorts,
                  std::shared_ptr<odometry::AbstractOdometry> odometry,
                  std::shared_ptr<controller::AbstractController> controller)
      : m_odometry(std::move(odometry)),
        m_controller(std::move(controller)),
        m_leftDrive(leftPorts),
        m_rightDrive(rightPorts),
        m_brakeType(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST)
  {
  }

 public:  // Methods
  // Driving types
  virtual void tank(std::int8_t leftJoyPct, std::int8_t rightJoyPct) = 0;

  // Ew
  virtual void arcade(std::int8_t fwdPct, std::int8_t turnPct) = 0;

  // BrakeType
  pros::motor_brake_mode_e_t getBrakeType() { return this->m_brakeType; }
  void setBrakeType(pros::motor_brake_mode_e_t newBrakeType)
  {
    this->m_brakeType = newBrakeType;
  }

 public:  // Overrides
  // Local Motions
  virtual void turnTo(utilities::Angle Heading, bool relative, bool reverse = false,
                      bool thru = false,
                      double timeout = std::numeric_limits<double>::infinity(),
                      bool async = false) = 0;
  virtual void moveFor(double inches, bool thru = false,
                       double timeout = std::numeric_limits<double>::infinity(),
                       bool async = false) = 0;

  // X, Y Motion
  virtual void goToPoint(utilities::Point point, bool thru = false,
                         double timeout = std::numeric_limits<double>::infinity(),
                         bool async = false) = 0;
  virtual void turnToPoint(utilities::Point point, bool reverse = false,
                           bool thru = false,
                           double timeout = std::numeric_limits<double>::infinity(),
                           bool async = false) = 0;

  // X, Y, Heading Pathing Motion
  virtual void goToPose(utilities::Pose pose, bool thru = false,
                        double timeout = std::numeric_limits<double>::infinity(),
                        bool async = false) = 0;
  virtual void turnToPose(utilities::Pose pose, bool reverse = false, bool thru = false,
                          double timeout = std::numeric_limits<double>::infinity(),
                          bool async = false) = 0;
};
}  // namespace lib2131::chassis