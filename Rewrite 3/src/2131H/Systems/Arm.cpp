/**
 * @file Arm.hpp
 * @author Andrew Hilton (2131H)
 * @brief Arm System Definition
 * @version 0.1
 * @date 2024-12-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "2131H/Systems/Arm.hpp"

#include <cmath>

#include "2131H/Utilities/Average.hpp"

void Arm::setPosition(double newPosition) { m_targetPosition = newPosition; }
double Arm::getPosition()
{
  if (m_isMotorEncoder)  // If we are using just a motor
  {
    m_pMotor->set_encoder_units(pros::MotorEncoderUnits::degrees);
    return Utilities::avg(m_pMotor->get_position_all()) * m_ratio;
  }
  // Using a rotational sensor for position
  else { return std::fmod((m_pRotational->get_position() / 100.0) * m_ratio, 360); }
}

void Arm::setIndex(size_t newIndex)
{
  m_macroIndex = std::clamp(
      static_cast<int>(newIndex),
      0,
      static_cast<int>(m_macroPositions.size()) - 1);  // Make sure new position is valid
  this->setPosition(m_macroPositions[m_macroIndex]);   // Move arm to desired position
}

int Arm::getIndex() { return m_macroIndex; }

Arm::Arm(
    pros::MotorGroup* pMotors,
    pros::Rotation* pRotation,
    double ratio,
    std::vector<double> armPositions,
    pros::controller_digital_e_t upButton,
    pros::controller_digital_e_t downButton,
    pros::Controller* pController,
    lemlib::PID* armPID)
    : m_pMotor(pMotors),
      m_pRotational(pRotation),
      m_isMotorEncoder(pRotation == nullptr),
      m_ratio(ratio),
      m_macroPositions(std::move(armPositions)),
      m_upBtnDetector(upButton, pController),
      m_downBtnDetector(downButton, pController),
      m_pid(armPID),
      m_enabled(true),
      m_thread(
          [this]() {
            while (true)
            {
              this->_update();
              pros::delay(10);
            }
          },
          "Arm Thread")
{
  m_pRotational->set_data_rate(5);
}

void Arm::_update()
{
  if (m_enabled)
  {
    // Calculate Error
    double error = (m_targetPosition - this->getPosition());

    // Enforce Shortest Path
    if (error < -270) { error += 360; }

    // Update PID and move by Output
    m_pMotor->move_voltage(m_pid->update(error * 100.0));
  }
}

void Arm::teleOp()
{
  // Update Buttons
  m_upBtnDetector.update();
  m_downBtnDetector.update();

  // Update macro indexes
  if (m_upBtnDetector.changedToPressed())
  {
    this->setIndex(this->getIndex() + 1);
    if (m_macroIndex == 1) { this->enable(); }
  }
  else if (m_downBtnDetector.changedToPressed()) { this->setIndex(this->getIndex() - 1); }

  // if (m_macroIndex == 0) { this->setPosition(-10); }
}

void Arm::disable() { this->m_enabled = false; }
void Arm::enable() { this->m_enabled = true; }