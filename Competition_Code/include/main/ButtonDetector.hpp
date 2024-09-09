#pragma once

#include "ChangeDetector.hpp"
#include "main/RobotConfig.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

class ButtonDetector : ChangeDetector<bool>
{
 private:  // Variables
  pros::controller_digital_e_t m_button;

 public:  // Constructors
  ButtonDetector(pros::controller_digital_e_t button) : m_button(button) {}

 public:  // Functions
  /**
   * @brief Get whether a button's state changed from released to pressed
   * @note getChanged and getValue are inherited from the ChangeDetector class.
   *
   * @return true Button Changed from released to pressed
   * @return false Button didn't change or changed from pressed to released
   */
  bool changedToPressed() { return this->getChanged() && this->getValue(); }
  /**
   * @brief Get whether a button's state changed from pressed to released
   * @note getChanged and getValue are inherited from the ChangeDetector class.
   *
   * @return true Button Changed from pressed to released
   * @return false Button didn't change or change from released to pressed
   */
  bool changedToReleased() { return this->getChanged() && !this->getValue(); }
  /**
   * @brief Get whether a button's state is pressing
   * @note this->getValue() is inherited from the ChangeDetector class
   *
   * @return true Button is pressing
   * @return false Button isn't pressing
   */
  bool isPressing() { return this->getValue(); }
  /**
   * @brief Get whether a button's state is released
   * @note this->getValue() is inherited from the ChangeDetector class
   *
   * @return true Button is released
   * @return false Button isn't released
   */
  bool isReleased() { return !this->getValue(); }

  /**
   * @brief Update Button Detector. This must be run every loop to update the
   * ChangeDetector and get new outputs.
   *
   */
  void update() { this->check(primary.get_digital(m_button)); }
};
