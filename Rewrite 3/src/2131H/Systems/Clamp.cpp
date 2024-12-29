/**
 * @file Clamp.hpp
 * @author Andrew Hilton (2131H)
 * @brief Clamp Class Definition
 * @version 0.1
 * @date 2024-11-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "2131H/Systems/Clamp.hpp"

bool Clamp::isGoal() { return m_goalDetected; }

bool Clamp::isActivated() { return m_pPneumatic->is_extended(); }

bool Clamp::isClamped() { return m_pPneumatic->is_extended() && m_goalDetected; }

void Clamp::enableAutoClamp() { m_autoClampEnabled = true; }

void Clamp::disableAutoClamp(bool dropGoal)
{
  m_autoClampEnabled = false;
  if (dropGoal) this->deactivate();
}

void Clamp::activate() { m_pPneumatic->extend(); }

void Clamp::deactivate() { m_pPneumatic->retract(); }

Clamp::Clamp(
    pros::adi::Pneumatics* pPneumatic,
    pros::Distance* pDistance,
    int triggerDistance,
    pros::controller_digital_e_t button,
    pros::Controller* pController,
    bool autoClampEnabled)
    : m_pPneumatic(pPneumatic),
      m_pDistance(pDistance),
      m_autoClampEnabled(autoClampEnabled),
      m_goalDetected(false),
      m_pController(pController),
      m_triggerDistance(triggerDistance),
      toggleBtnDetector(button, pController),
      m_thread(
          [this]() {
            while (true)
            {
              this->_update();
              pros::delay(10);
            }
          },
          "Clamp Thread")
{
}

void Clamp::_update()
{
  // Check if a goal is detected
  m_goalDetected = (m_pDistance->get_distance() < m_triggerDistance);

  // If a goal is detected then clamp
  if (m_autoClampEnabled && this->isGoal()) { this->activate(); }
}

void Clamp::teleOp()
{
  disableAutoClamp(false);

  // Update Detector
  toggleBtnDetector.update();

  // If button pressed, toggle clamp
  if (toggleBtnDetector.changedToPressed())
  {
    m_pPneumatic->toggle();
    if (this->isClamped()) { m_pController->rumble("."); }
  }
}
