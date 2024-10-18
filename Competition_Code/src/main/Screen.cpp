#include "main/Screen.hpp"

#include "Autonomous.hpp"
#include "ChangeDetector.hpp"
#include "RobotConfig.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
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
    {"LOW STAKE", "(48, 12, -180)", Autonomous::lowStake},    // lowStake Autonomous
    {"High Stake", "(24, 20, -180)", Autonomous::highStake},  // highStake Autonomous
    {"SKILLS", "Skills Description", Autonomous::skills},     // Skills Autonomous
    {"DEBUG", "Debug Description", Autonomous::debug},        // Debug for PID etc
};
int index = -1;  // Index of card (Increments by +1 on initial)

bool debug(true);                     // Enable / Disable Debug Output on the screen
bool initial(true);                   // Whether the screen has been initialized;
ChangeDetector<bool> ScreenDetector;  // ChangeDetector for the screen touch status
ChangeDetector<bool> TeamDetector;
bool redTeam(0);

/**
 * @brief Update the brain screen. Must be called for Screen to be drawn.
 *
 */
void update()
{
  // Update change detectors
  ScreenDetector.check(pros::screen::touch_status().touch_status == pros::E_TOUCH_PRESSED);
  TeamDetector.check(teamColor.get_value());

  if (TeamDetector.getChanged() && TeamDetector.getValue())
  {
    redTeam = !redTeam;
    Cards[index].draw(redTeam);  // Draw the card
  }

  // If ScreenDetector detects the button changed from released to pressed
  // or it's the inital loop
  if (ScreenDetector.getChanged() && ScreenDetector.getValue() || initial)
  {
    index++;                    // Increment Index
    if (index >= Cards.size())  // If Index is too large, then rollover to the start
    {
      index = 0;
    }
    Cards[index].draw(redTeam);  // Draw the card
    // Print battery life
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN, HEIGHT - MEDIUM_TEXT_MARGIN - 2 * MEDIUM_TEXT_SIZE, "Robot Battery %: %f",
                        pros::battery::get_capacity());
  }

  // If debug is enabled
  if (debug)
  {
    // Print robot position
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN, HEIGHT - MEDIUM_TEXT_MARGIN - MEDIUM_TEXT_SIZE, "X: %f, Y: %f, Theta: %f",
                        pose.x, pose.y, pose.theta);
  }

  // No long the inital execution of Screen::update()
  initial = false;
}

AutonCard* getAuton() { return &Cards[index]; }
bool isRedTeam() { return redTeam; }

pros::Task ScreenThread([]() {
  pros::delay(10);
  while (true)
  {
    update();
    pros::delay(50);
  }
});

}  // namespace Screen