/**
 * @file Clamp.hpp
 * @author Andrew Hilton (2131H)
 * @brief Clamp Class Declaration
 * @version 0.1
 * @date 2024-11-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "2131H/Utilities/ButtonDetector.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"

class Clamp
{
 private:                                               // *** ==== Member Variables ==== *** //
  std::shared_ptr<pros::adi::Pneumatics> m_pPneumatic;  // - Pointer to Pneumatics
  std::shared_ptr<pros::v5::Distance> m_pDistance;      // - Pointer to Distance sensor
                                                        //
  bool m_autoClampEnabled;                              // - Is autoClampEnabled?
  bool m_goalDetected;                                  // - Is a goal detected?
  int m_triggerDistance;                                // - Minimum Distance for a
                                                        // goal to be counted as detected
  Utilities::ButtonDetector toggleBtnDetector;          // - State checker for Controller Behavior
  pros ::Controller* m_pController;                     // - Pointer to Controller
                                                        //
  pros::Task m_thread;                                  // - Runs _update() in a thread
  float const m_indentDistance;

 public:  // *** ==== Getters / Setters ==== *** //
  /**
   * @brief Is a goal detected by the distance sensor?
   *
   * @return true
   * @return false
   */
  bool isGoal();

  /**
   * @brief Is the clamp activated?
   *
   * @return true
   * @return false
   */
  bool isActivated();

  /**
   * @brief Is a goal detected and clamp activated?
   *
   * @return true
   * @return false
   */
  bool isClamped();

  /**
   * @brief Enable Auto-Clamp
   *
   */
  void enableAutoClamp();

  /**
   * @brief Disable Auto-Clamp
   *
   * @param dropGoal Drop goal upon disabling?
   */
  void disableAutoClamp(bool dropGoal = true);

  /**
   * @brief Activate Clamp
   *
   */
  void activate();

  /**
   * @brief Deactivate Clamp
   *
   */
  void deactivate();

 public:  // *** ==== Constructors / Deconstructors ==== *** //
  /**
   * @brief Construct a new Clamp object
   *
   * @param clampPort Clamp port 'a' - 'h'
   * @param distancePort Distance port 0 - 21
   * @param goalIndent Sensor distance from edge of robot in inches
   * @param triggerDistance Distance measured by distance sensor before clamping
   * @param button Button for toggling the clamp
   * @param pController Controller for clamp Ex: &primary
   * @param autoClampEnabled Is Auto-Clamp enabled?
   */
  Clamp(
      pros::adi::Pneumatics* pPneumatic,
      pros::Distance* pDistance,
      int triggerDistance,
      float goalIndent,
      pros::controller_digital_e_t button,
      pros::Controller* pController,
      bool autoClampEnabled = false);

 protected:  // *** ==== Protected Methods ==== *** //
  /**
   * @brief Background update function
   *
   */
  void _update();

 public:  // *** ==== Public Methods ==== *** //
  /**
   * @brief Tele-Operator for the Clamp
   *
   */
  void teleOp();
  const float getGoalIndent();
};