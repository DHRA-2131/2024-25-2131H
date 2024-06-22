#pragma once

#include <math.h>

#include <cstdint>
#include <limits>

#include "lib2131/chassis/AbstractChassis.hpp"
#include "lib2131/utilities/Motion.hpp"
#include "lib2131/utilities/MotionProfile.hpp"
#include "pros/rtos.hpp"

namespace lib2131::chassis
{

class DifferentialChassis : public AbstractChassis
{
 private:  // Variables
  ChassisParameters m_params;

 public:  // Constructors
  DifferentialChassis(ChassisParameters params,
                      const std::initializer_list<std::int8_t> leftPorts,
                      const std::initializer_list<std::int8_t> rightPorts,
                      std::shared_ptr<odometry::AbstractOdometry> odometry,
                      std::shared_ptr<controller::AbstractController> controller)
      : AbstractChassis(leftPorts, rightPorts, odometry, controller), m_params(params)
  {
  }

 public:  // Overrides
  void tank(std::int8_t leftJoyPct, std::int8_t rightJoyPct) override
  {
    m_leftDrive.move_voltage(leftJoyPct / 100 * 12000);
    m_rightDrive.move_voltage(rightJoyPct / 100 * 12000);
  }
  void arcade(std::int8_t fwdPct, std::int8_t turnPct) override
  {
    m_leftDrive.move_voltage(fwdPct + turnPct / 100 * 12000);
    m_rightDrive.move_voltage(fwdPct - turnPct / 100 * 12000);
  }
  // Local Motions
  void turnTo(utilities::Angle Heading, bool relative, bool reverse = false,
              bool thru = false, double timeout = std::numeric_limits<double>::infinity(),
              bool async = false) override
  {
    if (async)
    {
      pros::Task asyncTask([&]() { turnTo(Heading, relative, false); });
    }
    else
    {
      // Set Target
      this->m_controller->setAngularTarget(Heading, this->m_odometry->getState().Position,
                                           relative);

      // Generate Motion Profile
      utilities::MotionProfile mp(Heading.getRadians(), m_params.maxAngularVelocity,
                                  m_params.maxAngularAcceleration,
                                  m_params.maxAngularDeceleration);
      // Timer
      uint msec = 0;
      // While exit conditions not met
      while (!m_controller->canExit() && msec < timeout)
      {
        // Get Controller Output
        utilities::Motion controllerOut = m_controller->getAngleOutput(
            this->m_odometry->getState().Position.theta, reverse, thru, 10);

        // Get Motion Profile Output
        double mpOut = mp.getVelocity(msec);

        double output;
        // Average them together
        if (mpOut != 0) { output = (mpOut + controllerOut.AngularVelocity) / 2; }
        else { output = controllerOut.AngularVelocity; }

        // Convert to Pct then convert to Voltage (mV)
        this->m_leftDrive.move_voltage(output / m_params.maxAngularVelocity * 12000);
        this->m_rightDrive.move_voltage(-output / m_params.maxAngularVelocity * 12000);

        // Allow for other tasks to finish and motors to update
        pros::delay(10);
        // Update Counter
        msec += 10;
      }
    }
  }

  void moveFor(double inches, bool thru = false,
               double timeout = std::numeric_limits<double>::infinity(),
               bool async = false) override
  {
    if (async)
    {
      pros::Task asyncTask([&]() { moveFor(inches, thru, timeout, false); });
    }
    else
    {
      // Set Target
      this->m_controller->setTarget({inches, 0, {0, false}},
                                    this->m_odometry->getState().Position, true);

      // Go backwards?
      const bool reverse(inches < 0);

      // Generate Motion Profile
      utilities::MotionProfile mp(inches, m_params.maxLinearVelocity,
                                  m_params.maxLinearAcceleration,
                                  m_params.maxLinearDeceleration);
      // Timer
      uint msec = 0;
      // While exit conditions not met
      while (!m_controller->canExit() && msec < timeout)
      {
        // Get Controller Output
        utilities::Motion controllerOut = m_controller->getOutput(
            this->m_odometry->getState().Position, reverse, thru, 10);

        // Get Motion Profile Output
        double mpOut = mp.getVelocity(msec);

        double output;
        // Average them together
        if (mpOut != 0) { output = (mpOut + controllerOut.LinearVelocity) / 2; }
        else { output = controllerOut.LinearVelocity; }

        // Convert to Pct then convert to Voltage (mV)
        output = output / m_params.maxLinearVelocity * 12000;
        this->m_leftDrive.move_voltage(output);
        this->m_rightDrive.move_voltage(output);

        // Allow for other tasks to finish and motors to update
        pros::delay(10);
        // Update Counter
        msec += 10;
      }
    }
  }

  // X, Y Motion
  void goToPoint(utilities::Point point, bool relative, bool thru = false,
                 double timeout = std::numeric_limits<double>::infinity(),
                 bool async = false) override
  {
    if (async)
    {
      pros::Task asyncTask([&]() { goToPoint(point, relative, thru, timeout, false); });
    }
    else
    {
      utilities::Pose currentPose(this->m_odometry->getState().Position);
      // Set Target
      this->m_controller->setTarget({point.x, point.y, {0, false}}, currentPose, false);

      // Go backwards?
      const bool reverse =
          fabs(this->m_controller->getAngularError(currentPose.theta).getRadians()) >
          M_PI_2;

      // Generate Motion Profile
      utilities::MotionProfile linearMp(
          currentPose.magnitude({point.x, point.y, {0, false}}),
          m_params.maxLinearVelocity, m_params.maxLinearAcceleration,
          m_params.maxLinearDeceleration);

      utilities::MotionProfile angularMp(
          m_controller->getAngularError(currentPose.theta).getRadians(),
          m_params.maxAngularVelocity, m_params.maxAngularAcceleration,
          m_params.maxAngularDeceleration);

      // Timer
      uint msec = 0;
      // While exit conditions not met
      while (!m_controller->canExit() && msec < timeout)
      {
        // Get Controller Output
        utilities::Motion controllerOut = m_controller->getOutput(
            this->m_odometry->getState().Position, reverse, thru, 10);

        // Get Motion Profile Output
        utilities::Motion mpOut = {linearMp.getVelocity(msec),
                                   angularMp.getVelocity(msec)};

        utilities::Motion output;
        // Average them together
        if (mpOut.LinearVelocity != 0 || mpOut.AngularVelocity != 0)
        {
          output = {(mpOut.LinearVelocity + controllerOut.LinearVelocity) / 2,
                    (mpOut.AngularVelocity + controllerOut.AngularVelocity) / 2};
        }
        else { output = controllerOut; }

        // Convert to Pct then convert to Voltage (mV)
        double leftOutput = (output.LinearVelocity / m_params.maxLinearVelocity +
                             output.AngularVelocity / m_params.maxAngularVelocity) *
                            12000;
        double rightOutput = (output.LinearVelocity / m_params.maxLinearVelocity -
                              output.AngularVelocity / m_params.maxAngularVelocity) *
                             12000;

        this->m_leftDrive.move_voltage(leftOutput);
        this->m_rightDrive.move_voltage(rightOutput);

        // Allow for other tasks to finish and motors to update
        pros::delay(10);
        // Update Counter
        msec += 10;
      }
    }
  }

  void turnToPoint(utilities::Point point, bool reverse = false, bool thru = false,
                   double timeout = std::numeric_limits<double>::infinity(),
                   bool async = false) override
  {
  }

  // X, Y, Heading Pathing Motion
  void goToPose(utilities::Pose pose, bool relative, bool thru = false,
                double timeout = std::numeric_limits<double>::infinity(),
                bool async = false) override
  {
  }
  void turnToPose(utilities::Pose pose, bool reverse = false, bool thru = false,
                  double timeout = std::numeric_limits<double>::infinity(),
                  bool async = false) override
  {
  }
};

}  // namespace lib2131::chassis