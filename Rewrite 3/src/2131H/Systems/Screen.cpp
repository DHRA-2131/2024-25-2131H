#include "2131H/Systems/Screen.hpp"

#include "2131H/Utilities/ChangeDetector.hpp"
#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"
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
void NULL_AUTON(bool) {}

// Initialization of a list Containing AutonCards.
// Each Card can be customized with a name and description.
std::vector<AutonCard> Cards = {
    {"NO AUTO", "", NULL_AUTON},  //
    {"DEBUG", "", Autonomous::debug},
    {"GOAL SIDE (QUAL)", "", Autonomous::goalSide},
    {"RING SIDE (QUAL)", "", Autonomous::ringSide},
    {"SOLO AWP (QUAL)", "F*CK", Autonomous::soloAWP},
    {"5 Ring Side (ELIMS)", "I Quit", Autonomous::ringSideFive},
    {"5 Goal Side (ELIMS)", "I'm Done", Autonomous::goalSideFive},
    {"GOAL RUSH (ELIMS)", "<WASTED>", Autonomous::goalRush}
};

int index = 6;  // Index of card (Increments by +1 on initial)

bool debug(true);                     // Enable / Disable Debug Output on the screen
bool initial(true);                   // Whether the screen has been initialized;
ChangeDetector<bool> ScreenDetector;  // ChangeDetector for the screen touch status
ChangeDetector<bool> TeamDetector;    // ChangeDetector for the team color status
bool redTeam(true);                   // If current team color is red / blue

/**
 * @brief Update the brain screen. Must be called for Screen to be drawn.
 *
 */
void update()
{
  // Check if screen touch status has changed
  ScreenDetector.check(cycleAuton.get_value());

  // Check if Team Color status has changed
  TeamDetector.check(teamColor.get_value());

  // If the teamColor button state just changed to true
  if (TeamDetector.getChanged() && TeamDetector.getValue())
  {
    redTeam = !redTeam;          // Toggle team color
    Cards[index].draw(redTeam);  // Redraw screen
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
    pros::screen::print(
        pros::E_TEXT_MEDIUM,
        MEDIUM_TEXT_MARGIN,
        HEIGHT - MEDIUM_TEXT_MARGIN - 2 * MEDIUM_TEXT_SIZE,
        "Robot Battery %: %f",
        pros::battery::get_capacity());
  }

  // If debug is enabled
  if (debug)
  {
    // Print robot position
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(
        pros::E_TEXT_MEDIUM,
        MEDIUM_TEXT_MARGIN,
        HEIGHT - MEDIUM_TEXT_MARGIN - MEDIUM_TEXT_SIZE,
        "X: %f, Y: %f, Theta: %f",
        pose.x,
        pose.y,
        pose.theta);
  }

  // No long the inital execution of Screen::update(), so initial = false;
  initial = false;
}

/**
 * @brief Get the AutonCard of current screen
 *
 * @return AutonCard*
 */
AutonCard* getAuton() { return &Cards[index]; }

/**
 * @brief Is the current team color red
 *
 * @return true Yes, it's red
 * @return false No, it's blue
 */
bool isRedTeam() { return redTeam; }

// Screen thread
pros::Task ScreenThread([]() {
  pros::delay(10);  // Delay for os
  while (true)
  {
    update();         // Run update task
    pros::delay(50);  // Don't take up CPU Resources
  }
});

}  // namespace Screen