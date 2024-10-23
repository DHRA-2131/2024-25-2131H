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
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"

using namespace Utilities;

namespace Systems
{

struct ChassisParameters
{
  pros::MotorGroup* leftDrive;
  pros::MotorGroup* rightDrive;

  const double wheelRpm;
  const double motorRpm;

  const double wheelSize;
  const double trackWidth;
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

  // Theoretical Max Velocity (In / S)
  const double m_maxVelocity;

  // Positional Information (In, In, Degrees)
  Pose m_currentPose;
  Pose m_prevPose;

  // Velocity Commands
  double m_angularVelocity;  // (In / S)
  double m_linearVelocity;   // (Rad / S)

  // Motor Voltages [-12000 mV, 12000 mV]
  double m_leftPct;
  double m_rightPct;

  // Odometry Instance
  AbstractOdometry* m_odometry;
  bool m_chassisCalibrated;  // Have we calibrated?

  // Chassis Threading
  pros::Task ChassisThread;
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
        m_maxVelocity(m_chassisInfo.wheelRpm * m_chassisInfo.wheelSize * M_PI / 60.0),
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
                if (m_odometryEnabled &&
                    m_chassisCalibrated)  // Only calculate if Enabled and Calibrated
                {
                  m_prevPose = std::move(m_currentPose);  // Store previous Positions
                  m_currentPose =
                      m_odometry->updatePose(this->m_currentPose);  // Update Odometry Position
                }

                if (m_chassisEnabled)  // Allow for Motor Control
                {
                  // Right Wheel (In / S) = (linear + angular)
                  // Left Wheel (In / S) = (linear - angular)
                  // Velocity Command (% of Max) =  Left Wheel, Right Wheel / Max Velocity
                  m_rightPct = (m_linearVelocity + m_angularVelocity) / m_maxVelocity;
                  m_leftPct = (m_linearVelocity - m_angularVelocity) / m_maxVelocity;

                  // Cap Motor Velocities
                  m_rightPct = std::clamp(m_rightPct, -100.0, 100.0);
                  m_leftPct = std::clamp(m_leftPct, -100.0, 100.0);

                  // TODO: Slew Rate?

                  // Update Motors
                  m_chassisInfo.leftDrive->move_velocity(m_rightPct * m_chassisInfo.motorRpm);
                  m_chassisInfo.rightDrive->move_velocity(m_leftPct * m_chassisInfo.motorRpm);
                }
                pros::delay(10);  // Don't take CPU resources
              }
            },
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Chassis Thread")
  {
  }

 private:  // === Member Functions === //
  ;

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
  double getMaxVelocity() { return m_maxVelocity; }
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