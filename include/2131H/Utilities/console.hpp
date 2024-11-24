/**
 * @file console.hpp
 * @author Andrew Hilton (2131H)
 * @brief Declaration for the console logging system
 * @version 0.1
 * @date 2024-10-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <iostream>
#include <sstream>

#include "2131H/Utilities/pose.hpp"
#include "pros/rtos.hpp"

namespace Utilities
{
class Logger
{
 private:                                 // === Member Variables  === //
  bool m_enabled;                         // Enable and Disables all logs
  std::ostringstream m_oss;               // Single output buffer
  pros::Task m_thread;                    // Thread for logger
  constexpr static int refreshRate = 10;  // Refresh rate of Logger

 public:  // === Public Variables === //
  // Foreground Colors
  static constexpr const char* FG_Default = "\033[39m";
  static constexpr const char* FG_Black = "\033[30m";
  static constexpr const char* FG_Red = "\033[31m";
  static constexpr const char* FG_Green = "\033[32m";
  static constexpr const char* FG_Yellow = "\033[33m";
  static constexpr const char* FG_Blue = "\033[34m";
  static constexpr const char* FG_Magenta = "\033[35m";
  static constexpr const char* FG_Cyan = "\033[36m";
  static constexpr const char* FG_Light_Gray = "\033[37m";
  static constexpr const char* FG_Dark_Gray = "\033[90m";
  static constexpr const char* FG_Light_Red = "\033[91m";
  static constexpr const char* FG_Light_Green = "\033[92m";
  static constexpr const char* FG_Light_Yellow = "\033[93m";
  static constexpr const char* FG_Light_Blue = "\033[94m";
  static constexpr const char* FG_Light_Magenta = "\033[95m";
  static constexpr const char* FG_Light_Cyan = "\033[96m";
  static constexpr const char* FG_White = "\033[97m";

  // Background Colors
  static constexpr const char* BG_Default = "\033[49m";
  static constexpr const char* BG_Black = "\033[40m";
  static constexpr const char* BG_Red = "\033[41m";
  static constexpr const char* BG_Green = "\033[42m";
  static constexpr const char* BG_Yellow = "\033[43m";
  static constexpr const char* BG_Blue = "\033[44m";
  static constexpr const char* BG_Magenta = "\033[45m";
  static constexpr const char* BG_Cyan = "\033[46m";
  static constexpr const char* BG_Light_Gray = "\033[47m";
  static constexpr const char* BG_Dark_Gray = "\033[100m";
  static constexpr const char* BG_Light_Red = "\033[101m";
  static constexpr const char* BG_Light_Green = "\033[102m";
  static constexpr const char* BG_Light_Yellow = "\033[103m";
  static constexpr const char* BG_Light_Blue = "\033[104m";
  static constexpr const char* BG_Light_Magenta = "\033[105m";
  static constexpr const char* BG_Light_Cyan = "\033[106m";
  static constexpr const char* BG_White = "\033[107m";

  // Text Characteristics
  static constexpr const char* Reset = "\033[0m";
  static constexpr const char* Underlined = "\033[21m";

 public:  // === Constructors === //
  Logger(bool enabled)
      : m_enabled(enabled), m_thread([this]() {
          while (true)
          {
            this->_writeBuffer();
            pros::delay(refreshRate);
          }
        })
  {
    // UnSync with printf and scanf as logger only uses std::cout.
    // This improves performance on logs as it reduces backwards compatibility overhead.
    std::ios::sync_with_stdio(false);
  }

 public:  // === Functions === //
  /**
   * @brief logs information to terminal.
   *
   * @tparam Args Arguments (Any Type, variadic function)
   * @param args Arguments (Any Type, variadic function)
   */
  template <typename... Args>
  void log(Args... args)
  {
    this->m_oss << Logger::Reset;  // Reset coloring
    this->m_oss << "log:";         // mark as a log
    (this->m_oss << ... << args);  // write each argument (args) to buffer string (fast)
    this->m_oss << ";";            // mark end of log
  }

  void logPose(const char* id, const Pose& pose)
  {
    this->m_oss << Logger::Reset;         // Reset coloring
    this->m_oss << id;                    // Add ID
    this->m_oss << ':';                   // Add Separator
    this->m_oss << pose.x;                // X Pose
    this->m_oss << ",";                   // Add Separator
    this->m_oss << pose.y;                // Y Pose
    this->m_oss << ",";                   // Add Separator
    this->m_oss << pose.getTheta(false);  // Theta (Degrees)
    this->m_oss << ";";                   // mark end of log
  }

  /**
   * @brief Adds a new line to the terminal
   *
   */
  void newLine() { std::cout << "\n"; }

  /**
   * @brief Enable logger output
   *
   */
  void enable() { m_enabled = true; }
  /**
   * @brief Disable logger output
   *
   */
  void disable() { m_enabled = false; }

 private:
  /**
   * @brief Writes buffer to cout
   *
   */
  void _writeBuffer()
  {
    // (Ideally only enable when debugging as to give max performance during matches)
    if (m_enabled && (this->m_oss.str() != ""))  // Only log if enabled
    {
      this->m_oss << Reset;  // Reset coloring
      std::cout << "$" + this->m_oss.str()
                << std::endl;  // One write to the cout (Slow, but only one cout call)
      this->m_oss.str("");
      this->m_oss.clear();  // clear buffer
    }
  }
};
}  // namespace Utilities

Utilities::Logger Console(true);