/**
 * @file Arm.hpp
 * @author Andrew Hilton (2131H)
 * @brief Arm System Declaration
 * @version 0.1
 * @date 2024-11-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstddef>
#include <memory>

#include "2131H/Utilities/ButtonDetector.hpp"
#include "lemlib/pid.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class Arm
{
 private:                                             // *** ==== Member Variables ==== *** //
  std::shared_ptr<pros::v5::MotorGroup> m_pMotor;     // - Pointer to MotorGroup
  std::shared_ptr<pros::v5::Rotation> m_pRotational;  // - Pointer to Rotational
                                                      //
  std::vector<double> m_macroPositions;               // - Target Positions for Driver Macro
  int m_macroIndex;                                   // - Current Index for macro'd positions
                                                      //
  double m_ratio;                                     // - Ratio from positional sensor to arm
  double m_targetPosition = 0;                        // - Target Position of Arm
                                                      //
  bool m_isMotorEncoder;                              // - Using a Rotational or Motor?
  bool m_isMoving;                                    // - Is the arm in motion
  bool m_enabled;                                     // - Is the arm enabled?
                                                      //
  Utilities::ButtonDetector m_upBtnDetector;          // - State Checker for Controller behavior
  Utilities::ButtonDetector m_downBtnDetector;        // - State CHecker for Controller behavior
                                                      //
  lemlib::PID* m_pid;                                 // - Pointer to PID Controller for the arm
                                                      //
  pros::Task m_thread;                                // - Runs _update() in a thread

 public:  // *** ==== Getters / Setters ==== *** //
  /**
   * @brief Set the target position of the arm
   *
   * @param newPosition Target position of the arm in degrees
   */
  void setPosition(double newPosition);

  /**
   * @brief Get the actual position of the arm
   *
   * @return double
   */
  double getPosition();

  /**
   * @brief Set the position macro index
   *
   * @param newIndex New index value
   */
  void setIndex(size_t newIndex);

  /**
   * @brief Get the current index of the arm
   *
   * @return int
   */
  int getIndex();

 public:  // *** ==== Constructors / Deconstructors ==== *** //
  /**
   * @brief Construct a new Arm
   *
   * @param motorPorts All Ports in a {}, negative denotes reversal of a motor
   * @param cartridgeType Cartridge type of all motors
   * @param rotationalPort Rotational Port Number
   * @param reversed Is Rotational reversed
   * @param ratio Ratio from Rotational to Arm
   * @param armPositions Target Positions for the Driver Macro in a {}
   * @param upButton Ex: pros::E_CONTROLLER_DIGITAL_L1
   * @param downButton Ex: pros::E_CONTROLLER_DIGITAL_L2
   * @param controller Pointer to controller. Ex: &primary
   * @param armPID Pointer to the Arm PID Controller
   */
  Arm(pros::MotorGroup* pMotors,
      pros::Rotation* pRotation,
      double ratio,
      std::vector<double> armPositions,
      pros::controller_digital_e_t upButton,
      pros::controller_digital_e_t downButton,
      pros::Controller* pController,
      lemlib::PID* armPID);

 private:  // *** ==== Private Methods ==== *** //
  /**
   * @brief Background Update Function
   *
   */
  void _update();

 public:  // *** ==== Public Methods ==== *** //
  /**
   * @brief Tele-Operator for Arm
   *
   */
  void teleOp();

  void disable();
  void enable();
};
