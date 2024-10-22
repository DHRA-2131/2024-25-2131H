#pragma once

#include <string>

#include "pros/screen.h"
#include "pros/screen.hpp"

namespace Screen
{
extern bool debug;

typedef void (*callback)(bool);

#define WIDTH 480    // Width of the screen
#define HEIGHT 240   // Height of the screen
#define PEN_WIDTH 1  // Pen width

#define LARGE_TEXT_MARGIN 10  // Margin for large text
#define LARGE_TEXT_SIZE 30    // Large text size in pnt

#define MEDIUM_TEXT_MARGIN 10  // Margin for medium text
#define MEDIUM_TEXT_SIZE 20    // Medium text size in pnt

/**
 * @brief Container for Autonomous Information. Used in Screen.hpp.
 *
 */
class AutonCard
{
 private:                   // Variables
  std::string m_name;       // Name of Auton
  std::string m_setupDesc;  // Set up Description of Auton

  void (*m_pAutonCallback)(bool);  // Callback to Auton

 public:  // Constructors
  /**
   * @brief Construct a new Auton Card
   *
   * @param name Name of Autonomous Route
   * @param setupDesc Set-Up Description
   * @param AutonCallback Callback to the Autonomous Function
   */
  AutonCard(std::string name, std::string setupDesc, void (*AutonCallback)(bool))
      : m_name(name), m_setupDesc(setupDesc), m_pAutonCallback(AutonCallback)
  {
  }

 public:  // Methods
  /**
   * @brief Draw AutonCard on screen
   *
   */
  void draw(bool redTeam)
  {
    if (redTeam)  // If on red team
    {
      // Draw background in red
      pros::screen::set_eraser(0xff0000);  // Red
    }
    else
    {
      // Draw background in blue
      pros::screen::set_eraser(0x0000ff);  // Blue
    }
    // Set Pen color to white
    pros::screen::set_pen(0xffffff);  // White

    // Clear Screen
    pros::screen::erase_rect(0, 0, WIDTH, HEIGHT);

    // Draw Background
    pros::screen::draw_rect(PEN_WIDTH, PEN_WIDTH, WIDTH - PEN_WIDTH, HEIGHT - PEN_WIDTH);
    // Draw Frame around text
    pros::screen::draw_rect(PEN_WIDTH, PEN_WIDTH, WIDTH - PEN_WIDTH, 2 * LARGE_TEXT_MARGIN + LARGE_TEXT_SIZE + PEN_WIDTH);

    // Display Name
    pros::screen::print(pros::E_TEXT_LARGE, LARGE_TEXT_MARGIN, LARGE_TEXT_MARGIN, m_name.c_str());
    // Display Set-Up Description
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN, 2 * LARGE_TEXT_MARGIN + LARGE_TEXT_SIZE + PEN_WIDTH + MEDIUM_TEXT_MARGIN,
                        m_setupDesc.c_str());
  }

  /**
   * @brief Get the Name of AutonCard
   *
   * @return std::string
   */
  std::string getName() { return m_name; }

  /**
   * @brief Get the Auton Callback from AutonCard
   *
   * @return callback
   */
  callback getAutonCB() { return m_pAutonCallback; }
};

/**
 * @brief Update the screen
 *
 */
void update();

/**
 * @brief Return if team color is red
 *
 * @return true Yes, it's red
 * @return false No, it's blue
 */
bool isRedTeam();

// Get AutonCard (Returns current auton card)
AutonCard* getAuton();

}  // namespace Screen