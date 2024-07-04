#pragma once

#include <memory>

#include "lib2131/chassis/ChassisParameters.hpp"
#include "lib2131/controller/AbstractController.hpp"
#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/utilities/Point.hpp"
#include "lib2131/utilities/Units.h"
#include "pros/misc.hpp"
#include "pros/motors.h"

namespace lib2131::chassis
{

class AbstractChassis
{
 protected:  // Variables
  ChassisParameters m_chassisConstraints;
  std::shared_ptr<controller::AbstractController> m_controller;

  pros::motor_brake_mode_e m_brakeType;

 public:  // Variables
  std::shared_ptr<odometry::AbstractOdometry> m_odometry;

 public:  // Constructors
  AbstractChassis(ChassisParameters chassisConstraints,
                  std::shared_ptr<odometry::AbstractOdometry> odometry,
                  std::shared_ptr<controller::AbstractController> controller)
      : m_chassisConstraints(chassisConstraints),
        m_odometry(std::move(odometry)),
        m_controller(std::move(controller)),
        m_brakeType(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST)
  {
  }

 public:  // Methods
  // Driving types
  virtual void tank(const pros::Controller& Master) = 0;

  // Ew
  virtual void arcade(const pros::Controller& Master) = 0;

  // BrakeType
  pros::motor_brake_mode_e_t getBrakeType() { return this->m_brakeType; }

 public:  // Overrides
  virtual void setBrakeType(pros::motor_brake_mode_e_t newBrakeType) = 0;

  // Local Motions
  virtual void turnTo(units::angle::radian_t Heading, bool relative, bool reverse = false,
                      bool thru = false,
                      units::time::second_t timeout =
                          units::time::second_t(std::numeric_limits<double>::infinity()),
                      bool async = false) = 0;

  virtual void moveFor(double inches, bool thru = false,
                       units::time::second_t timeout =
                           units::time::second_t(std::numeric_limits<double>::infinity()),
                       bool async = false) = 0;

  // X, Y Motion
  virtual void goToPoint(utilities::Point point, bool relative, bool thru = false,
                         units::time::second_t timeout = units::time::second_t(
                             std::numeric_limits<double>::infinity()),
                         bool async = false) = 0;

  virtual void turnToPoint(utilities::Point point, bool reverse = false,
                           bool thru = false,
                           units::time::second_t timeout = units::time::second_t(
                               std::numeric_limits<double>::infinity()),
                           bool async = false) = 0;

  // X, Y, Heading Pathing Motion
  virtual void goToPose(utilities::Pose pose, bool relative, bool thru = false,
                        units::time::second_t timeout = units::time::second_t(
                            std::numeric_limits<double>::infinity()),
                        bool async = false) = 0;

  virtual void turnToPose(utilities::Pose pose, bool reverse = false, bool thru = false,
                          units::time::second_t timeout = units::time::second_t(
                              std::numeric_limits<double>::infinity()),
                          bool async = false) = 0;
};
}  // namespace lib2131::chassis