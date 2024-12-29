/**
 * @file Console.hpp
 * @author Andrew Hilton (2131H)
 * @brief Console Class Defintion
 * @version 0.1
 * @date 2024-12-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "2131H/Utilities/Console.hpp"

#include <iostream>

#include "pros/rtos.hpp"


Terminal::Terminal(int refreshRate)
    : m_refreshRate(refreshRate), m_usingGUI(false), m_thread([this]() {
        while (true)
        {
          this->_update();
          pros::Task::delay(m_refreshRate);
        }
      })
{
}

void Terminal::_update()
{
  if (m_oss.str() != "")
  {
    if (m_usingGUI) { std::cout << "$"; }
    std::cout << m_oss.str() << std::endl;
    m_oss.clear();
  }
}
template <typename T>
void Terminal::log(T message)
{
  if (m_usingGUI) { m_oss << m_logFlag; }
  m_oss << message;
  if (!m_usingGUI) { this->endPrint(); }
}

void Terminal::endPrint()
{
  if (m_usingGUI) { m_oss << m_delimiter << '\n'; }
  else { m_oss << '\n'; }
}

void Terminal::useGUI(bool usingGUI) { m_usingGUI = usingGUI; }