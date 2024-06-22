#pragma once

#include <cstdint>

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
  // Local Motions
  void turnTo(utilities::Angle Heading, bool relative, bool reverse = false,
              bool thru = false, bool async = false) override
  {
    if (async)
    {
      pros::Task asyncTask([&]() { turnTo(Heading, relative, false); });
    }
    else
    {
      // Generate Motion Profile
      utilities::MotionProfile mp(Heading.getRadians(), m_params.maxAngularVelocity,
                                  m_params.maxAngularAcceleration,
                                  m_params.maxAngularDeceleration);

      uint msec = 0;
      // While exit conditions not met
      while (!m_controller->canExit())
      {
        // Get Controller Output
        utilities::Motion controllerOut = m_controller->getAngleOutput(
            this->m_odometry->getState().Position.theta, reverse, thru, 10);

        // Get Motion Profile Output
        utilities::Motion mpOut = {0, mp.getVelocity(msec)};

        double output;
        // Average them together
        if (mpOut.AngularVelocity != 0)
        {
          output = (mpOut.AngularVelocity + controllerOut.AngularVelocity) / 2;
        }
        else { output = controllerOut.AngularVelocity; }

        this->m_leftDrive.move_voltage(output / m_params.maxAngularVelocity * 12000);
        this->m_rightDrive.move_voltage(-output / m_params.maxAngularVelocity * 12000);

        pros::delay(10);
        msec += 10;
      }
    }
  }
  void moveFor(double inches, bool thru = false, bool async = false) override {}

  // X, Y Motion
  void goToPoint(utilities::Point point, bool thru = false, bool async = false) override
  {
  }
  void turnToPoint(utilities::Point point, bool reverse = false, bool thru = false,
                   bool async = false) override
  {
  }

  // X, Y, Heading Pathing Motion
  void goToPose(utilities::Pose pose, bool thru = false, bool async = false) override {}
  void turnToPose(utilities::Pose pose, bool reverse = false, bool thru = false,
                  bool async = false) override
  {
  }
};

}  // namespace lib2131::chassis