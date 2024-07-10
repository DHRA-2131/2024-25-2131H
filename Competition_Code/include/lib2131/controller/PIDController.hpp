#pragma once

#include <cmath>
#include <memory>

#include "lib2131/controller/AbstractController.hpp"
#include "lib2131/controller/ExitCondition.hpp"
#include "lib2131/utilities/MotionOutputs.hpp"
#include "lib2131/utilities/PID.hpp"
#include "lib2131/utilities/Pose.hpp"

namespace lib2131::controller
{

class PIDController : public AbstractController
{
  using angle_t = units::angle::radian_t;
  using distance_t = units::length::inch_t;
  using velocity =
      units::compound_unit<units::length::inches, units::inverse<units::time::seconds>>;
  using velocity_t = units::unit_t<velocity>;

  utilities::PID m_linearPID, m_angularPID;
  distance_t m_minError;
  bool m_canReverse;

  double m_minVelocity;
  bool m_turnOvershoot;

 public:
  PIDController(exit_condition::ExitCondition &exitConditionals, utilities::PID linearPID,
                utilities::PID angularPID)
      : AbstractController(exitConditionals),
        m_linearPID(linearPID),
        m_angularPID(angularPID)
  {
  }

  utilities::WheelVelocities get_command(int deltaTime, utilities::Pose currentPose,
                                         bool reverse, bool thru)
  {
    int dir = reverse ? -1 : 1;

    bool chainedExecutable = false;

    distance_t distance_error = currentPose.pos.magnitude(m_target.pos);

    angle_t angle_error = currentPose.pos.amplitude(m_target.pos) - currentPose.theta;

    if (reverse)
    {
      angle_error = currentPose.pos.amplitude(m_target.pos * -1) - currentPose.theta;
    }

    angle_error = units::math::fmod(angle_error, angle_t(2 * M_PI));

    velocity_t lin_speed =
        (thru ? velocity_t(100.0)
              : velocity_t((m_linearPID.calc(distance_error.value(), deltaTime))) * dir);

    double ang_speed;
    if (distance_error < m_minError)
    {
      this->m_canReverse = true;
      // reduce the linear speed if the bot is tangent to the m_target
      lin_speed *= units::math::cos(angle_error);
    }
    else if (distance_error < 2 * m_minError)
    {
      // scale angular speed down to 0 as distance_error approaches m_minError
      ang_speed = m_angularPID.update(angle_error);
      ang_speed *= (distance_error - m_minError) / m_minError;
    }
    else
    {
      if (fabs(angle_error) > M_PI_2 && this->m_canReverse)
      {
        angle_error = angle_error - (angle_error / fabs(angle_error)) * M_PI;
        lin_speed = -lin_speed;
      }

      ang_speed = m_angularPID.update(angle_error);
    }
    lin_speed = std::max(-100.0, std::min(100.0, lin_speed));
    // Runs at the end of a through movement
    if (ec->is_met(this->l->get_pose(), thru))
    {
      if (thru)
      {
        return chassis::DiffChassisCommand{chassis::diff_commands::Chained{
            dir * std::fmax(lin_speed, this->m_minVelocity) - ang_speed,
            dir * std::fmax(lin_speed, this->m_minVelocity) + ang_speed}};
      }
      else { return chassis::Stop{}; }
    }

    return chassis::DiffChassisCommand{
        chassis::diff_commands::Voltages{lin_speed - ang_speed, lin_speed + ang_speed}};
  }

  chassis::DiffChassisCommand PIDController::get_angular_command(
      bool reverse, bool thru, lib2131::AngularDirection direction,
      std::shared_ptr<AbstractExitCondition> ec)
  {
    // Runs in the background of turn commands
    // Error is the difference between the m_target angle and the current angle
    // ArcTan is used to find the angle between the robot and the m_target
    // position Output is proportional to the error and the weight of the
    // constant Controller exits when robot settled for a designated time
    // duration or accurate to a specified tolerance
    double current_angle = this->l->get_orientation_rad();
    double m_target_angle = 0;
    if (!this->m_target.pos.theta.has_value())
    {
      Point currentPose = this->l->get_position();
      double dx = this->m_target.pos.x - currentPose.pos.x;
      double dy = this->m_target.pos.y - currentPose.pos.y;
      m_target_angle = atan2(dy, dx);
    }
    else { m_target_angle = this->angular_m_target; }
    double angular_error = m_target_angle - current_angle;
    bool chainedExecutable = false;
    angular_error = lib2131::norm_delta(angular_error);

    if (fabs(angular_error) < lib2131::to_radians(5))
    {
      if (thru) { chainedExecutable = true; }
      m_turnOvershoot = true;
    }

    if (!m_turnOvershoot)
    {
      if (direction == lib2131::AngularDirection::COUNTERCLOCKWISE && angular_error < 0)
      {
        angular_error += 2 * M_PI;
      }
      else if (direction == lib2131::AngularDirection::CLOCKWISE && angular_error > 0)
      {
        angular_error -= 2 * M_PI;
      }
    }

    double ang_speed = m_angularPID.update(angular_error);
    if (ec->is_met(this->l->get_pose(), thru) || chainedExecutable)
    {
      if (thru)
      {
        return chassis::DiffChassisCommand{
            chassis::diff_commands::Chained{-ang_speed, ang_speed}};
      }
      return chassis::Stop{};
    }
    return chassis::DiffChassisCommand{
        chassis::diff_commands::Voltages{-ang_speed, ang_speed}};
  }

  // Function to reset every variable used in the PID controller
  void reset()
  {
    this->m_linearPID.reset();
    this->m_angularPID.reset();
    this->m_canReverse = false;
    this->m_turnOvershoot = false;
  }

  std::shared_ptr<PIDController> modify_linear_constants(double kP, double kI, double kD);
  std::shared_ptr<PIDController> modify_angular_constants(double kP, double kI,
                                                          double kD);
  std::shared_ptr<PIDController> modify_m_minError(double m_minError);

  friend class PIDControllerBuilder;
  friend class BoomerangControllerBuilder;
};

}  // namespace lib2131::controller
