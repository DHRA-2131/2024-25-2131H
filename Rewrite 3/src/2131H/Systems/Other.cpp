/**
 * @file Other.hpp
 * @author Andrew Hilton (2131H)
 * @brief Other systems such as Doinkler
 * @version 0.1
 * @date 2024-12-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "2131H/Systems/Other.hpp"

Doinkler::Doinkler(
    std::uint8_t port,
    bool startExtended,
    pros::controller_digital_e_t button,
    pros::Controller* controller)
    : pros::adi::Pneumatics(port, startExtended), toggleBtnDetector(button, controller)
{
}

void Doinkler::teleOp()
{
  toggleBtnDetector.update();
  if (toggleBtnDetector.changedToPressed()) { this->toggle(); }
}

RingRush::RingRush(
    std::uint8_t port1,
    std::uint8_t port2,
    pros::controller_digital_e_t button,
    pros::Controller* controller,
    bool startExtended)
    : m_pPneumatic1(std::make_shared<pros::adi::Pneumatics>(port1, startExtended)),
      m_pPneumatic2(std::make_shared<pros::adi::Pneumatics>(port2, startExtended)),
      toggleBtnDetector(button, controller)
{
}

void RingRush::teleOp()
{
  toggleBtnDetector.update();
  if (toggleBtnDetector.changedToPressed())
  {
    m_pPneumatic1->toggle();
    m_pPneumatic2->toggle();
  }
}

void RingRush::extend()
{
  m_pPneumatic1->extend();
  m_pPneumatic2->extend();
}

void RingRush::retract()
{
  m_pPneumatic1->retract();
  m_pPneumatic2->retract();
}