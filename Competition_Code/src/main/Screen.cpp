#include "main/Screen.hpp"

#include "ChangeDetector.hpp"
#include "RobotConfig.hpp"
#include "pros/misc.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

/**
 * @brief The Screen namespace holds everything needed for our functional Brain Screen
 * selector
 *
 */
namespace Screen
{
/**
 * @brief NULL_AUTON is a autonomous that does nothing
 * @note This is intended to be used in the AutonCard class as a Callback that does
 * nothing
 */
void NULL_AUTON() {}

// Initialization of a list Containing AutonCards.
// Each Card can be customized with a name and description.
std::vector<AutonCard> Cards = {
    {"LEFT", "Left Description", NULL_AUTON},      // Left Field Autonomous [Not Coded]
    {"RIGHT", "Right Description", NULL_AUTON},    // Right Field Autonomous [Not Coded]
    {"SKILLS", "Skills Description", NULL_AUTON},  // Skills Autonomous [Not Coded]
    {"DEBUG", "Debug Description",
     NULL_AUTON},  // Debug (For Odom testing, PID Tunning, etc) [Not Coded]
    {"AUTON5", "AUTON5", NULL_AUTON}};  // Place holder autonomous for demonstration

bool debug(true);                     // Enable / Disable Debug Output on the screen
bool initial(true);                   // Whether the screen has been initialized;
int index = -1;                       // Index of card (Increments to 0 on initial)
ChangeDetector<bool> ScreenDetector;  // ChangeDetector for the screen touch status

/**
 * @brief Update the brain screen. Must be called for Screen to be drawn.
 *
 */
void update()
{
  // Update change detector
  ScreenDetector.check(pros::screen::touch_status().touch_status ==
                       pros::E_TOUCH_PRESSED);

  // If ScreenDetector detects the button changed from released to pressed
  // or it's the inital loop
  if (ScreenDetector.getChanged() && ScreenDetector.getValue() || initial)
  {
    index++;                    // Increment Index
    if (index >= Cards.size())  // If Index is too large, then rollover to the start
    {
      index = 0;
    }
    Cards[index].draw();  // Draw the card
  }

  // If debug is enabled
  if (debug)
  {
    // Print robot position
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN,
                        HEIGHT - MEDIUM_TEXT_MARGIN - MEDIUM_TEXT_SIZE,
                        "X: %f, Y: %f, Theta: %f", pose.x, pose.y, pose.theta);
    // Print battery life
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN,
                        HEIGHT - MEDIUM_TEXT_MARGIN - 2 * MEDIUM_TEXT_SIZE,
                        "Robot Battery %: %f", pros::battery::get_capacity());
  }

  // No long the inital execution of Screen::update()
  initial = false;
}
}  // namespace Screen