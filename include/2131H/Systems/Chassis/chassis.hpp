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

#include "2131H/Systems/Odometry/abstract-odometry.hpp"
#include "2131H/Utilities/console.hpp"
#include "2131H/Utilities/pose.hpp"
#include "pid-controller.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"

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
  int speed = 127;
};
struct turnToParams
{
  int speed = 127;
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

  // Odometry Instance
  AbstractOdometry* m_odometry;
  bool m_chassisCalibrated;  // Have we calibrated?

  // Chassis Threading
  pros::Task ChassisThread;
  const double deltaTime = 10;
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
      // Right Wheel (In / S) = (linear + angular)
      // Left Wheel (In / S) = (linear - angular)
      double newRight = (m_linearVelocity + m_angularVelocity);
      double newLeft = (m_linearVelocity - m_angularVelocity);

      // Calculate Acceleration of motors
      // (In / S) / 10 Ms * 100.0 = (In / S^2)
      double rightAccel = (newRight - m_right) * 1000.0 / deltaTime;
      double leftAccel = (newLeft - m_left) * 1000.0 / deltaTime;

      // If accel is too quick, limit to max accel
      if (rightAccel > m_chassisInfo.maxAcceleration)
      {
        // Set to max accel (Adjusted to (In/S) / DeltaTime)
        m_right += m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }
      else if (rightAccel < -m_chassisInfo.maxAcceleration)
      {
        m_right -= m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }
      else { m_right = newRight; }

      // If accel/deccel is too quick, limit to max accel/deccel
      if (leftAccel > m_chassisInfo.maxAcceleration)
      {
        m_left += m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }
      else if (leftAccel < -m_chassisInfo.maxAcceleration)
      {
        m_left -= m_chassisInfo.maxAcceleration * deltaTime / 1000.0;
      }
      else { m_left = newLeft; }

      // Cap Motor Velocities
      m_right = std::clamp(newRight, -m_chassisInfo.maxVelocity, m_chassisInfo.maxVelocity);
      m_left = std::clamp(newLeft, -m_chassisInfo.maxVelocity, m_chassisInfo.maxVelocity);

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
        m_chassisInfo.trackWidth / 2.0 *
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
    Pose out = (m_currentPose - m_prevPose) * 100.0;
    out.setTheta(out.getTheta(true) * 100.0, true);  // Also scale theta
    return out;
  }

 public:  // *** === Motions === *** //
  void moveToPoint(Pose target, int timeout, moveToParams params = {}, bool async = false) {}
  void moveForward(double distance, int timeout, bool forceHeading = true, moveToParams params = {},
                   bool async = false)
  {
  }
  void turnToPoint(Pose target, int timeout, turnToParams params = {}, bool async = false) {}
  void turnToHeading(double heading, int timeout, turnToParams params = {}, bool async = false) {}
  void swingToPoint(Pose target, int timeout, turnToParams params = {}, bool async = false) {}
  void swingToHeading(double heading, int timeout, turnToParams params = {}, bool async = false) {}
};
}  // namespace Systems