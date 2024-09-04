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
  bool changedToPressed() { return this->getChanged() && this->getValue(); }
  bool changedToReleased() { return this->getChanged() && !this->getValue(); }
  bool isPressing() { return this->getValue(); }
  bool isReleased() { return !this->getValue(); }

  void update() { this->check(primary.get_digital(m_button)); }
};