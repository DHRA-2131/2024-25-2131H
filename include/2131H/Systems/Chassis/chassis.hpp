/**
 * @file chassis.hpp
 * @author Andrew Hilton (2131H)
 * @brief Class Declaration for the Chassis. Uses a Localizer and controllers to move the chassis to
 * a position.
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <cmath>

#include "2131H/Systems/Odometry/abstract-odometry.hpp"
#include "2131H/Utilities/change-detector.hpp"
#include "2131H/Utilities/console.hpp"
#include "2131H/Utilities/pose.hpp"
#include "pid-controller.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

using namespace Utilities;

namespace Systems
{

struct ChassisParameters
{
  pros::MotorGroup* leftDrive;   // Pointer to left drive MotorGroup
  pros::MotorGroup* rightDrive;  // Pointer to left drive MotorGroup

  const double wheelRpm;  // max RPM of the wheels
  const double motorRpm;  // max RPM of the motor

  const double wheelSize;   // (Inches)
  const double trackWidth;  // (Inches)

  const double robotWeight;  // (Pounds)

  // === Overridable === //
  const double kTraction = 1;  // Traction tuning value for adjusting max Acceleration

  // === Calculated === //
  const double wheelSizeM = wheelSize / 39.3701;                     // (Meters)
  const double motorCount = leftDrive->size() + rightDrive->size();  // Total Count of all motors
  const double robotWeightKg = robotWeight * 0.45359237;             // (Kilograms)
  const double motorNm = 0.35 * (600.0 / motorRpm);                  // Nm of each motor
  const double maxVelocity = wheelRpm * wheelSize * M_PI / 60.0;     // Max Velocity (In/s)
  const double maxAcceleration = (motorCount * motorNm * motorRpm) /
                                 (wheelRpm * robotWeightKg * wheelSizeM) *
                                 kTraction;  // Max Acceleration of motors (In/S^2)
};

struct moveToParams
{
  bool forwards = true;   // Should the robot go forwards (defaults to true)
  double maxSpeed = 100;  // Robot Maximum speed (% of max Velocity)
  double minSpeed = 0;    // Robot Minimum speed (% of max Velocity)
};
struct turnToParams
{
  bool forwards = true;  // should the robot go forwards (defaulted to true)
  int maxSpeed = 100;    // Robot Maximum speed (% of max Velocity)
  int minSpeed = 0;      // Robot Minimum speed (% of max Velocity)
};

class Chassis
{
 private:  //  === Member Variables === //
  // Drive Information
  const ChassisParameters m_chassisInfo;

  // Positional Information (In, In, Degrees)
  Pose m_currentPose;
  Pose m_prevPose;

  // Velocity Commands
  double m_angularVelocity;  // (In / S)
  double m_linearVelocity;   // (Rad / S)

  // Motor Commands
  double m_left;
  double m_right;

  // Drive locking
  bool m_lockLeft = false;
  bool m_lockRight = false;

  ChangeDetector<bool> m_leftLockDetector;
  ChangeDetector<bool> m_rightLockDetector;

  // Odometry Instance
  AbstractOdometry* m_odometry;
  bool m_chassisCalibrated;  // Have we calibrated?

  // Chassis Threading
  pros::Task ChassisThread;
  const int deltaTime = 10;
  bool m_odometryEnabled;
  bool m_chassisEnabled;

 public:  // === Public Variables === //
  // Controllers
  PID* lateralPID;
  PID* angularPID;  //! Needs to update in degrees for user side to use degrees

 public:  // === Getters / Setters === //
  /**
   * @brief Set the Pose of the Chassis
   *
   * @param newPose
   */
  void setPose(const Pose& newPose) { m_currentPose = newPose; }

  /**
   * @brief Get the Pose of the Chassis
   *
   * @return const Pose&
   */
  const Pose& getPose() { return m_currentPose; }

 public:  // === Constructors === //
  Chassis(ChassisParameters info, AbstractOdometry* Odometry, PID* lateralPID, PID* angularPID)
      : m_chassisInfo(info),
        m_odometryEnabled(true),
        m_chassisEnabled(true),
        m_odometry(Odometry),
        m_currentPose(0, 0, 0),
        m_prevPose(0, 0, 0),
        lateralPID(lateralPID),
        angularPID(angularPID),
        ChassisThread(
            [this]() {
              int count = 0;
              while (true)
              {
                _update(deltaTime);
                pros::delay(deltaTime);  // Don't take CPU resources
              }
            },
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Chassis Thread")
  {
  }

 private:  // === Member Functions === //
  /**
   * @brief Updates all chassis code, runs in ChassisThread
   *
   */
  void _update(double deltaTime)
  {
    if (m_odometryEnabled && m_chassisCalibrated)  // Only calculate if Enabled and Calibrated
    {
      m_prevPose = std::move(m_currentPose);                        // Store previous Positions
      m_currentPose = m_odometry->updatePose(this->m_currentPose);  // Update Odometry Position
    }

    if (m_chassisEnabled)  // Allow for Motor Control
    {
      // Convert angular velocity to a motor velocity
      double tangentVelocity = m_chassisInfo.trackWidth / 2.0 * m_angularVelocity;

      // Right Wheel (In / S) = (linear + angular)
      // Left Wheel (In / S) = (linear - angular)
      double newRight = (m_linearVelocity + tangentVelocity);
      double newLeft = (m_linearVelocity - tangentVelocity);

      // Calculate Acceleration of motors
      // (In / S) / 10 Ms * 100.0 = (In / S^2)
      double rightAccel = (newRight - m_right) * 1000.0 / deltaTime;
      double leftAccel = (newLeft - m_left) * 1000.0 / deltaTime;

      // If accel is too quick, limit to max accel
      if (rightAccel > m_chassisInfo.maxAcceleration)
      {
        // Set to max accel (Adjusted to (In/S) / DeltaTime)
        newRight += m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }
      else if (rightAccel < -m_chassisInfo.maxAcceleration)
      {
        newRight -= m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }

      // If accel/deccel is too quick, limit to max accel/deccel
      if (leftAccel > m_chassisInfo.maxAcceleration)
      {
        newLeft += m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }
      else if (leftAccel < -m_chassisInfo.maxAcceleration)
      {
        newLeft -= m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }

      // Cap motor velocities but keep the scaling between them
      const float ratio =
          std::max(std::fabs(newRight), std::fabs(newLeft)) / m_chassisInfo.maxVelocity;
      if (ratio > 1)
      {
        newRight /= ratio;
        newLeft /= ratio;
      }

      // Update motor commands with calculated and scaled commands
      m_right = newRight;
      m_left = newLeft;

      m_leftLockDetector.check(m_lockLeft);
      m_rightLockDetector.check(m_lockRight);

      // If left Drive is locked
      if (m_leftLockDetector.getValue() && m_leftLockDetector.getChanged())
      {
        // Lock once
        m_chassisInfo.leftDrive->set_brake_mode(pros::MotorBrake::hold);
      }
      // If it's changed to unlocked, unlock
      else if (!m_leftLockDetector.getValue() && m_leftLockDetector.getChanged())
      {
        // Unlock once
        m_chassisInfo.leftDrive->set_brake_mode(pros::MotorBrake::coast);
      }
      // If its locked, don't spin it
      else if (m_lockLeft) { m_left = 0; }

      // Right side
      if (m_rightLockDetector.getValue() && m_rightLockDetector.getChanged())
      {
        m_right = 0;
        m_chassisInfo.rightDrive->set_brake_mode(pros::MotorBrake::hold);
      }
      else if (!m_rightLockDetector.getValue() && m_rightLockDetector.getChanged())
      {
        m_chassisInfo.rightDrive->set_brake_mode(pros::MotorBrake::coast);
      }
      else if (m_lockRight) { m_right = 0; }

      // Update Motors
      // Motor Command / Max Velocity = % of max * Max RPM = Desired RPM
      m_chassisInfo.leftDrive->move_velocity(m_right / m_chassisInfo.maxVelocity *
                                             m_chassisInfo.motorRpm);
      m_chassisInfo.rightDrive->move_velocity(m_left / m_chassisInfo.maxVelocity *
                                              m_chassisInfo.motorRpm);
    }
  }

 public:  // === Functions === //
  /**
   * @brief Initializes the drive. Should be included in the main.cpp file.
   *
   */
  void init()
  {
    m_chassisCalibrated = true;
    // Calibrate Odometry
    m_odometry->init(true);
    this->resumeOdometry();
  }
  /**
   * @brief Suspends Odometry Calculations.
   *
   */
  void suspendOdometry() { m_odometryEnabled = false; }

  /**
   * @brief Resumes Odometry Calculations.
   *
   */
  void resumeOdometry() { m_odometryEnabled = true; }

  /**
   * @brief Is Odometry enabled / Calculating?
   *
   * @return true
   * @return false
   */
  bool odometryEnabled() { return m_odometryEnabled; }

  void logPosition(const char* color = Logger::Reset) const
  {
    Console.log("(", m_currentPose.x, ", ", m_currentPose.y, ", ", m_currentPose.getTheta(false),
                ")");
  }
  void logVelocity(const char* color = Logger::Reset)
  {
    auto velocity = this->getActualVelocities();
    Console.log("(", velocity.x, ", ", velocity.y, ", ", velocity.getTheta(false), ")");
  }
  /**
   * @brief Suspends motion from the Chassis thread. (Odometry will still calculate)
   *
   */
  void disable() { m_chassisEnabled = false; }

  /**
   * @brief Resumes motion from the Chassis thread. (Odometry will still calculate)
   *
   */
  void enable() { m_chassisEnabled = true; }

  /**
   * @brief Is chassis enabled / Calculating?
   *
   * @return true
   * @return false
   */
  bool chassisEnabled() { return m_chassisEnabled; }

  /**
   * @brief Suspends the Chassis thread (No Motor Updates, No Odometry)
   *
   */
  void suspendAll() { ChassisThread.suspend(); }
  /**
   * @brief Resumes the chassis thread (Enables Motor Updates and Odometry). ChassisThread starts
   * resumed.
   *
   */
  void resumeAll() { ChassisThread.resume(); }

 public:  // === Getters / Setters === //
  /**
   * @brief Returns the calculated maximum velocity
   *
   * @return double
   */
  double getMaxVelocity() { return m_chassisInfo.maxVelocity; }

  /**
   * @brief Returns the calculated maximum acceleration
   *
   * @return double
   */
  double getMaxAcceleration() { return m_chassisInfo.maxAcceleration; }

  /**
   * @brief Set Velocity of Drivetrain
   *
   * @param linearVelocity in (In/S)
   * @param angularVelocity in (deg / S)
   */
  void setVelocities(double linearVelocity, double angularVelocity, bool radians = false)
  {
    m_linearVelocity = linearVelocity;
    m_angularVelocity =
        (radians ? angularVelocity : angularVelocity * M_PI / 180.0);  // Force (Rad / S)
  }

  /**
   * @brief Get the commanded linear velocity.
   *
   * @return double
   */
  double getCommandedLinearVelocity() { return m_linearVelocity; }
  /**
   * @brief Get the commanded angular velocity
   *
   * @return double
   */
  double getCommandedAngularVelocity() { return m_angularVelocity; }

  /**
   * @brief Get the actual Velocity of the chassis (measured from odometry)
   *
   * @return Velocity Pose (In / S, In / S, Deg / S)
   */
  Pose getActualVelocities()
  {
    // Delta Position (In, In, Deg) / 10 msec
    // Velocity (In, In, Deg) / S = Delta Position * 100.0 ((Msec) -> (S))
    Pose out = (m_currentPose - m_prevPose) * (1000.0 / deltaTime);
    out.setTheta((m_currentPose.getTheta(true) - m_prevPose.getTheta(true)) * (1000.0 / deltaTime),
                 true);  // Also scale theta
    return out;
  }

 public:  // *** === Motions === *** //
  void moveToPoint(Pose target, int timeout, moveToParams params = {}, bool async = false)
  {
    if (async)  // If async start a async thread
    {
      pros::Task asyncThread([&, this]() { this->moveToPoint(target, timeout, params, false); },
                             "Async Move Forwards");
      pros::delay(10);  // Task needs some time to start
      return;           // Exit function
    }

    // Reset PIDs
    lateralPID->reset();
    angularPID->reset();

    int time = 0;  // Time of motion in milliseconds
    while (time < timeout &&
           (!lateralPID->exitConditionsMet(deltaTime) && !angularPID->exitConditionsMet(deltaTime)))
    {
      // Get the pose of the robot (marked as const as a precaution)
      const Pose currentPose = this->getPose();

      // Current theta (adjusted for forwards parameter, using radians)
      const double robotTheta =
          params.forwards ? currentPose.getTheta(true) : currentPose.getTheta(true) + M_PI;

      // Get angle error between two points (In radians)
      double angleError = currentPose.angle(target, true) - robotTheta;

      // Normalize the angle error to force the shortest turn direction
      if (angleError > M_PI) { angleError -= 2 * M_PI; }
      else if (angleError < -M_PI) { angleError += 2 * M_PI; }

      // Get lateral error (scale by cos angleError to enforce turning)
      double lateralError = currentPose.distance(target) * cos(angleError);

      // Calculate PID outputs and set velocity of the drive
      // TODO: limit angular and lateral by params.minSpeed and params.maxSpeed
      double lateralOut = lateralPID->getOutput(lateralError, deltaTime);
      double angularOut = angularPID->getOutput(angleError, deltaTime);

      this->setVelocities(lateralOut, angularOut);

      // Update time and delay to not take all the CPU's resources
      time += deltaTime;
      pros::delay(deltaTime);
    }
  }

  void moveForward(double distance, int timeout, bool forceHeading = true, moveToParams params = {},
                   bool async = false)
  {
    // When heading isn't enforced we just tell the motors to essentially move forwards
    if (!forceHeading)
    {
      if (async)  // If async start a async thread
      {
        pros::Task asyncThread(
            [&, this]() { this->moveForward(distance, timeout, forceHeading, params, false); },
            "Async Move Forwards");
        pros::delay(10);  // Task needs some time to start
        return;           // Exit function
      }

      //* There is no need to reset PIDs because this movement is open loop

      bool fwd = (distance > 0);  // Is the robot moving fwd or backwards
      distance = fabs(distance);  // Sanitize the distance for motion profile math

      // This is the minimum distance required for a trapezoidal motion profile
      double minDistance = std::pow(m_chassisInfo.maxVelocity, 2) / m_chassisInfo.maxAcceleration;
      // Calculates MP off of distance and chassis info, for more information visit:
      // https://www.desmos.com/calculator/daculs5px6

      // TODO: Account for Initial Velocity && .maxSpeed and .minSpeed scaling
      double accelTime = (m_chassisInfo.maxVelocity) / m_chassisInfo.maxAcceleration;
      double accelDistance = 0.5 * m_chassisInfo.maxAcceleration * std::pow(accelTime, 2);

      //* The linked graph accounts for a separate Deccel. For VRC this isn't needed because the
      //* Accel is roughly equal to Deccel
      double coastDistance = distance - accelDistance * 2.0;
      double coastTime = (coastDistance / m_chassisInfo.maxVelocity);
      double time = 0;      // Current time of motion //* Time is in seconds for motion profile math
      double velocity = 0;  // Velocity output
      double totalTime = coastTime + 2.0 * accelTime;

      if (distance > minDistance)  // Run a triangle motion profile because there is no coast
      {
        // In this situation coastTime is negative which is needed for calculations
        while (time < totalTime && time < timeout / 1000.0)
        {
          // For half of the total time accel
          if (time < totalTime / 2.0)
          {
            velocity += m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
          }
          // Otherwise, deccel until total time is hit
          else { velocity -= m_chassisInfo.maxAcceleration * deltaTime / 1000.0; }

          // if not forwards, We need to go backwards!
          if (!fwd) { velocity *= 1; }

          this->setVelocities(velocity, 0);  // Send motor commands

          time += deltaTime / 1000.0;  // Update time
          pros::delay(deltaTime);      // Delay
        }
      }
      else  // Use a Trapezoidal Motion Profile
      {
        while (time < totalTime && time < timeout / 1000.0)
        {
          // Accelerate by the max Acceleration if in accel phase
          if (time < accelTime) { velocity += m_chassisInfo.maxAcceleration * deltaTime / 1000.0; }
          else if (time > accelTime && time > coastTime + accelTime)
          {
            // Coast phase (Code should already coast so nothing is really needed here)
          }
          // Decelerate by the max Acceleration during the deccel phases
          else if (time > coastTime + accelTime)
          {
            velocity -= m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
          }

          // if not forwards, We need to go backwards!
          if (!fwd) { velocity *= 1; }

          setVelocities(velocity, 0);  // Send motor commands

          time += deltaTime / 1000.0;  // Update time
          pros::delay(deltaTime);      // Delay
        }
      }
    }
    else  // Otherwise, it will use Odometry to calculate a point and drive to the point
    {
      Pose currentPose = this->getPose();
      double currentTheta = currentPose.getTheta(true);

      // Calculate the target point in front / behind the bot
      Pose targetPose =
          currentPose + Pose{distance * cos(currentTheta), distance * sin(currentTheta), 0};
      this->moveToPoint(targetPose, timeout, params, async);  // Move to calculated point
    }
  }
  void turnToPoint(Pose target, int timeout, turnToParams params = {}, bool async = false)
  {
    // Reset angular PID Controller
    angularPID->reset();

    int time = 0;
    while (time < timeout &&
           (!lateralPID->exitConditionsMet(deltaTime) && !angularPID->exitConditionsMet(deltaTime)))
    {
      double targetHeading = this->getPose().angle(target);
      double robotTheta =
          params.forwards ? this->getPose().getTheta(true) : this->getPose().getTheta(true) + M_PI;
      double angleError = targetHeading - robotTheta;

      // Normalize the angle error to force the shortest turn direction
      if (angleError > M_PI) { angleError -= 2 * M_PI; }
      else if (angleError < -M_PI) { angleError += 2 * M_PI; }

      // Calculate angular PID output
      double angularOut = angularPID->getOutput(angleError, deltaTime);

      // Send command to motors
      this->setVelocities(0, angularOut);

      pros::delay(10);
      time += 10;
    }
  }
  void turnToHeading(double heading, int timeout, turnToParams params = {}, bool async = false)
  {
    // Reset angular PID Controller
    angularPID->reset();

    int time = 0;
    while (time < timeout &&
           (!lateralPID->exitConditionsMet(deltaTime) && !angularPID->exitConditionsMet(deltaTime)))
    {
      double robotTheta =
          params.forwards ? this->getPose().getTheta(true) : this->getPose().getTheta(true) + M_PI;
      double angleError = heading - robotTheta;

      // Normalize the angle error to force the shortest turn direction
      if (angleError > M_PI) { angleError -= 2 * M_PI; }
      else if (angleError < -M_PI) { angleError += 2 * M_PI; }

      // Calculate angular PID output
      double angularOut = angularPID->getOutput(angleError, deltaTime);

      // Send command to motors
      this->setVelocities(0, angularOut);

      pros::delay(10);
      time += 10;
    }
  }
  void swingToPoint(Pose target, int timeout, bool lockRightSide, turnToParams params = {},
                    bool async = false)
  {
    // Reset angular PID Controller
    angularPID->reset();

    int time = 0;
    while (time < timeout &&
           (!lateralPID->exitConditionsMet(deltaTime) && !angularPID->exitConditionsMet(deltaTime)))
    {
      double targetHeading = this->getPose().angle(target);
      double robotTheta =
          params.forwards ? this->getPose().getTheta(true) : this->getPose().getTheta(true) + M_PI;
      double angleError = targetHeading - robotTheta;

      // Lock drive side
      lockRightSide ? m_lockRight = true : m_lockLeft = true;

      // Normalize the angle error to force the shortest turn direction
      if (angleError > M_PI) { angleError -= 2 * M_PI; }
      else if (angleError < -M_PI) { angleError += 2 * M_PI; }

      // Calculate angular PID output
      double angularOut = angularPID->getOutput(angleError, deltaTime);

      // Send command to motors
      this->setVelocities(0, angularOut);

      pros::delay(10);
      time += 10;
    }
    // Unlock Drive side
    lockRightSide ? m_lockRight = false : m_lockLeft = false;
  }
  void swingToHeading(double heading, int timeout, bool lockRightSide, turnToParams params = {},
                      bool async = false)
  {
    // Reset angular PID Controller
    angularPID->reset();

    int time = 0;
    while (time < timeout &&
           (!lateralPID->exitConditionsMet(deltaTime) && !angularPID->exitConditionsMet(deltaTime)))
    {
      double robotTheta =
          params.forwards ? this->getPose().getTheta(true) : this->getPose().getTheta(true) + M_PI;
      double angleError = heading - robotTheta;

      // Normalize the angle error to force the shortest turn direction
      if (angleError > M_PI) { angleError -= 2 * M_PI; }
      else if (angleError < -M_PI) { angleError += 2 * M_PI; }

      // Lock a Drive side
      lockRightSide ? m_lockRight = true : m_lockLeft = true;

      // Calculate angular PID output
      double angularOut = angularPID->getOutput(angleError, deltaTime);

      // Send command to motors
      this->setVelocities(0, angularOut);

      pros::delay(10);
      time += 10;
    }
    // Unlock Drive side
    lockRightSide ? m_lockRight = false : m_lockLeft = false;
  }
};
}  // namespace Systems