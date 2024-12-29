/**
 * @file Console.hpp
 * @author Andrew Hilton (2131H)
 * @brief Console Class Declaration
 * @version 0.1
 * @date 2024-12-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <sstream>

#include "pros/rtos.hpp"

class Terminal
{
 private:  // *** ==== Member Variables ==== *** //
  std::ostringstream m_oss;
  int m_refreshRate;
  bool m_usingGUI;
  pros::Task m_thread;

  std::string m_logFlag = "log:";
  std::string m_delimiter = "|";

 public:  // *** ==== Public Constructor ==== *** //
  Terminal(int refreshRate = 10);

 private:
  void _update();

 public:  // *** ==== Public Methods ==== *** //
  /**
   * @brief Print a message to the console
   *
   * @tparam T
   * @param message
   */
  template <typename T>
  void log(T message);

  /**
   * @brief logs information to terminal.
   *
   * @tparam Args Arguments (Any Type, variadic function)
   * @param args Arguments (Any Type, variadic function)
   */
  template <typename... Args>
  void log(Args... args)
  {
    if (m_usingGUI) { this->m_oss << m_logFlag; }  // mark as a log
    (this->m_oss << ... << args);  // write each argument (args) to buffer string (fast)
    this->endPrint();              // end the log
  }

  /**
   * @brief Print a message to the console with a newline
   *
   */
  void endPrint();

  /**
   * @brief Set whether the Gui is connected or not
   *
   * @param usingGUI
   */
  void useGUI(bool usingGUI);
};