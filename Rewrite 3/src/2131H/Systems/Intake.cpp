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

void Intake::enableSort(RingColors sortColor)
{
  m_pOptical->set_integration_time(10);  // Decrease Update time

  m_sortEnabled = true;     // Enable Sort
  m_sortColor = sortColor;  // Set Sort Color
}
void Intake::disableSort()
{
  m_pOptical->set_led_pwm(0);  // Turn off light

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
    pros::MotorGroup* pMotors,
    pros::Optical* pOptical,
    pros::Distance* pDistance,
    pros::adi::Pneumatics* pPneumatics,
    std::int32_t sortDistance,
    pros::controller_digital_e_t intakeBtn,
    pros::controller_digital_e_t outtakeBtn,
    pros::Controller* pController,
    double redBound,
    double blueBound)
    : m_pMotors(pMotors),
      m_pOptical(pOptical),
      m_pDistance(pDistance),
      m_pPneumatics(pPneumatics),
      m_sortDistance(sortDistance),
      m_redBound(redBound),
      m_blueBound(blueBound),
      m_sortEnabled(false),
      m_sortColor(RingColors::NONE),
      m_colorStateDetector(Intake::RingColors::NONE),
      m_ringStateDetector(),
      intakeButton(intakeBtn, pController),
      outtakeButton(outtakeBtn, pController),
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

    // Calculate Current Ring Color
    if (color < m_redBound && proximity > 180) { ringColor = RingColors::RED; }
    else if (color > m_blueBound && proximity > 180) { ringColor = RingColors::BLUE; }
    else { ringColor = RingColors::NONE; }

    // Check if color changed
    m_colorStateDetector.check(ringColor);
    if (m_colorStateDetector.getChanged())
    {
      if (m_colorStateDetector.getValue() != RingColors::NONE)  // If color is not none
      {
        m_possession.push_back(m_colorStateDetector.getValue());  // Add new ring to possession
      }
    }

    std::int32_t distance = m_pDistance->get_distance();  // Grab Distance from distance sensor

    // Update Change Detector with new value
    m_ringStateDetector.check(distance < m_sortDistance);
    // If ring state has changed to detected
    if (m_ringStateDetector.getChanged() && m_ringStateDetector.getValue())
    {
      // If sort is active and color is wrong, remove ring
      if (m_sortEnabled && this->getCurrentRingColor() == m_sortColor) { m_pPneumatics->extend(); }
      if (m_possession.size() > 0)
      {
        m_possession.erase(m_possession.begin());  // Remove ring from count
      }
    }
    // If ring state has changed to nothing detected
    else if (m_ringStateDetector.getChanged() && !m_ringStateDetector.getValue())
    {
      // if the pneumatic is extened, retract it
      if (m_pPneumatics->is_extended())
      {
        pros::delay(100);
        m_pPneumatics->retract();
      }
    }
  }
  else if (!m_sortEnabled)
  {
    m_pOptical->set_led_pwm(0);  // Turn off light
  }
}

void Intake::spin(double voltage)
{
  m_pMotors->move_voltage(voltage);  // Spin intake at specified voltage
}

void Intake::stop()
{
  m_pMotors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  m_pMotors->brake();
  m_pMotors->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Intake::teleOp()
{
  intakeButton.update();
  outtakeButton.update();

  // If intake button is pressed, intake
  if (intakeButton.isPressing()) { this->spin(12000); }
  // Otherwise if outtake button is pressed, outtake
  else if (outtakeButton.isPressing()) { this->spin(-12000); }
  // Else, stop the intake
  else { this->stop(); }
}

void Intake::init() { m_initialized = true; }