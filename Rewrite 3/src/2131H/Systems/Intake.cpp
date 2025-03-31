/**
 * @file Intake.hpp
 * @author Andrew Hilton (2131H)
 * @brief Intake Class Defintion
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "2131H/Systems/Intake.hpp"

#include "pros/motors.h"
#include "pros/rtos.hpp"

void Intake::enableSort(RingColors sortColor)
{
  m_pOptical->set_integration_time(5);  // Decrease Update time

  m_sortEnabled = true;     // Enable Sort
  m_sortColor = sortColor;  // Set Sort Color
}
void Intake::disableSort()
{
  m_pOptical->set_led_pwm(0);  // Turn off light

  m_sorted = true;
  m_sortEnabled = false;  // Disable sort
}

Intake::RingColors Intake::getSortColor() { return m_sortColor; }
Intake::RingColors Intake::getCurrentRingColor()
{
  if (m_possession.size() > 0) { return m_possession[0]; }
  else { return RingColors::NONE; }
}

double Intake::getPossessionCount() { return m_possession.size(); }
std::vector<Intake::RingColors> Intake::getPossession() { return m_possession; }

Intake::Intake(
    pros::Motor* pFirstMotor,
    pros::Motor* pSecondMotor,
    pros::Optical* pOptical,
    pros::adi::Pneumatics* pLift,
    double sortDistance,
    pros::controller_digital_e_t intakeBtn,
    pros::controller_digital_e_t outtakeBtn,
    pros::controller_digital_e_t liftBtn,
    pros::Controller* pController,
    double redBound,
    double blueBound)
    : m_pFirstMotor(pFirstMotor),
      m_pSecondMotor(pSecondMotor),
      m_pOptical(pOptical),
      m_pLift(pLift),
      m_sortDistance(sortDistance),
      m_redBound(redBound),
      m_blueBound(blueBound),
      m_sortEnabled(false),
      m_sortColor(RingColors::NONE),
      m_colorStateDetector(Intake::RingColors::NONE),
      intakeButton(intakeBtn, pController),
      outtakeButton(outtakeBtn, pController),
      liftButton(liftBtn, pController),
      m_thread(
          [this]() {
            while (true)
            {
              this->_update();
              pros::delay(10);
            }
          },
          "Intake Thread")
{
}

void Intake::_update()
{
  if (m_initialized && m_sortEnabled)
  {
    m_pOptical->set_led_pwm(100);          // Turn on LED
    double color = m_pOptical->get_hue();  // Grab Color from sensor
    int proximity = m_pOptical->get_proximity();
    RingColors ringColor;

    m_position = m_pSecondMotor->get_position();

    // Calculate Current Ring Color
    if (color < m_redBound && proximity > 180) { ringColor = RingColors::RED; }
    else if (color > m_blueBound && proximity > 180) { ringColor = RingColors::BLUE; }
    else { ringColor = RingColors::NONE; }

    // Check if color changed
    m_colorStateDetector.check(ringColor);
    if (m_colorStateDetector.getChanged())
    {
      if (m_colorStateDetector.getValue() == m_sortColor)  // If color is not none
      {
        m_possession.push_back(m_colorStateDetector.getValue());  // Add new ring to possession
        std::cout << "RING COLOR DETECTED" << std::endl;
      }
    }

    // If sort is active and color is wrong, remove ring
    if (m_sortEnabled && this->getCurrentRingColor() == m_sortColor && m_sorted == true)
    {
      m_sorted = false;
      m_sortPosition = m_position + m_sortDistance;
    }

    if (m_possession.size() > 0 && m_position > m_sortPosition && m_sorted == false)
    {
      m_possession.erase(m_possession.begin());  // Remove ring from count
      m_pSecondMotor->move_voltage(-12000);
      pros::delay(750);
      std::cout << "RING COLOR SORTED" << std::endl;
      m_sorted = true;
    }
    else if (m_position < m_sortPosition && m_sorted == false)
    {
      m_pSecondMotor->move_voltage(12000);
    }
  }
  else if (!m_sortEnabled)
  {
    m_pOptical->set_led_pwm(0);  // Turn off light
  }
}

void Intake::spin(double voltage, uint stage)
{
  if (m_sorted)
  {
    if (stage > 1) {
      m_pFirstMotor->move_voltage(voltage);  // Spin intake at specified voltage
    } else if (stage > 2) {
      m_pSecondMotor->move_voltage(voltage);
    }
  }
}

void Intake::stop()
{
  m_pSecondMotor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  m_pFirstMotor->brake();
  m_pSecondMotor->brake();
}

void Intake::teleOp()
{
  intakeButton.update();
  outtakeButton.update();
  liftButton.update();

  // If intake button is pressed, intake
  if (intakeButton.isPressing()) { this->spin(12000); }
  // Otherwise if outtake button is pressed, outtake
  else if (outtakeButton.isPressing()) { this->spin(-12000); }
  // Else, stop the intake
  else { this->stop(); }

  if (liftButton.changedToPressed()) { this->toggleLift(); }
}

void Intake::init() { m_initialized = true; }
void Intake::lift() { m_pLift->extend(); }
void Intake::drop() { m_pLift->retract(); }
void Intake::toggleLift() { m_pLift->toggle(); }
bool Intake::isLifted() { return m_pLift->is_extended(); }