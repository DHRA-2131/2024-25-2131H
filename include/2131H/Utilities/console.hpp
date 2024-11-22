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

namespace Utilities
{
class Logger
{
 private:          // === Member Variables  === //
  bool m_enabled;  // Enable and Disables all logs

 public:  // === Public Variables === //
  // Foreground Colors
  static constexpr const char* FG_Default = "\x1B[39m";
  static constexpr const char* FG_Black = "\x1B[30m";
  static constexpr const char* FG_Red = "\x1B[31m";
  static constexpr const char* FG_Green = "\x1B[32m";
  static constexpr const char* FG_Yellow = "\x1B[33m";
  static constexpr const char* FG_Blue = "\x1B[34m";
  static constexpr const char* FG_Magenta = "\x1B[35m";
  static constexpr const char* FG_Cyan = "\x1B[36m";
  static constexpr const char* FG_Light_Gray = "\x1B[37m";
  static constexpr const char* FG_Dark_Gray = "\x1B[90m";
  static constexpr const char* FG_Light_Red = "\x1B[91m";
  static constexpr const char* FG_Light_Green = "\x1B[92m";
  static constexpr const char* FG_Light_Yellow = "\x1B[93m";
  static constexpr const char* FG_Light_Blue = "\x1B[94m";
  static constexpr const char* FG_Light_Magenta = "\x1B[95m";
  static constexpr const char* FG_Light_Cyan = "\x1B[96m";
  static constexpr const char* FG_White = "\x1B[97m";

  // Background Colors
  static constexpr const char* BG_Default = "\x1B[49m";
  static constexpr const char* BG_Black = "\x1B[40m";
  static constexpr const char* BG_Red = "\x1B[41m";
  static constexpr const char* BG_Green = "\x1B[42m";
  static constexpr const char* BG_Yellow = "\x1B[43m";
  static constexpr const char* BG_Blue = "\x1B[44m";
  static constexpr const char* BG_Magenta = "\x1B[45m";
  static constexpr const char* BG_Cyan = "\x1B[46m";
  static constexpr const char* BG_Light_Gray = "\x1B[47m";
  static constexpr const char* BG_Dark_Gray = "\x1B[100m";
  static constexpr const char* BG_Light_Red = "\x1B[101m";
  static constexpr const char* BG_Light_Green = "\x1B[102m";
  static constexpr const char* BG_Light_Yellow = "\x1B[103m";
  static constexpr const char* BG_Light_Blue = "\x1B[104m";
  static constexpr const char* BG_Light_Magenta = "\x1B[105m";
  static constexpr const char* BG_Light_Cyan = "\x1B[106m";
  static constexpr const char* BG_White = "\x1B[107m";

  // Text Characteristics
  static constexpr const char* Reset = "\x1B[0m";
  static constexpr const char* Underlined = "\x1B[21m";

 public:  // === Constructors === //
  Logger(bool enabled) : m_enabled(enabled)
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
    // (Ideally only enable when debugging as to give max performance during matches)
    if (m_enabled)  // Only log if enabled
    {
      std::ostringstream oss;               // Buffer String (Fast)
      (oss << ... << args);                 // write each argument (args) to buffer string
      oss << Reset;                         // Reset coloring
      std::cout << oss.str() << std::endl;  // One write to the cout (Slow, but only one cout call)
    }
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
};
}  // namespace Utilities

Utilities::Logger Console(true);