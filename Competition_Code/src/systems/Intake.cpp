#include "systems/Intake.hpp"

#include "main/ButtonConfig.hpp"
#include "main/ChangeDetector.hpp"
#include "main/RobotConfig.hpp"
#include "main/Screen.hpp"

namespace Systems
{
namespace Intake
{
bool enabled(1);
bool autoSortEnabled(1);

struct ringState  // All possible ring states
{
  static const int none = 0;  // No ring detected
  static const int red = 1;   // Red ring detected
  static const int blue = 2;  // Blue ring detected
};

int possession[2] = {ringState::none, ringState::none};         // Rings in intake as a array (starts with none)
ChangeDetector<int> PossessionChangeDetector(ringState::none);  // Change detection for Possession (Detects ring color)
ChangeDetector<bool> RingChangeDetector;                        // Change detection for auto sort (Detects change from distance sensor)
/**
 * @brief Enable auto color sort
 *
 */
void enableAutoSort()
{
  colorDetector.set_led_pwm(100);  // Increase light (gives better readings)
  autoSortEnabled = true;          // Enable auto sort
}
/**
 * @brief Disable color sort
 *
 */
void disableAutoSort()
{
  colorDetector.set_led_pwm(0);  // Turn off light
  autoSortEnabled = false;       // Disable auto sort
}

/**
 * @brief Operator control for the intake
 *
 */
void teleOp()
{
  // Enable Autosort going into driver
  disableAutoSort();
  if (enabled)  // if intake enabled
  {
    // If intake button pressing, intake
    if (Buttons::Intake.isPressing()) { motor.move_voltage(12000); }
    // If outtake button pressing, outtake
    else if (Buttons::Outtake.isPressing()) { motor.move_voltage(-12000); }
    // otherwise, stop intake
    else { motor.brake(); }
  }
}

// Calculate Possession
pros::Task possessionTask(
    []() {
      while (true)
      {
        // Get detected color
        auto color = colorDetector.get_hue();

        // Update Possession checker with ring color
        if (color < 20 && color != 0) { PossessionChangeDetector.check(ringState::red); }
        else if (color > 100) { PossessionChangeDetector.check(ringState::blue); }
        else { PossessionChangeDetector.check(ringState::none); }

        // Get ring color
        auto ring = PossessionChangeDetector.getValue();

        // If ring color changed, and the ring exists (red or blue), and the intake is spinning forward
        if (PossessionChangeDetector.getChanged() && (ring == ringState::red || ring == ringState::blue) &&
            motor.get_actual_velocity() > 10)
        {
          // If there are zero rings in the intake, add ring to first spot
          if (possession[0] == 0) { possession[0] = ring; }
          // If there is one ring in the intake, add ring to second spot
          else if (possession[1] == 0) { possession[1] = ring; }
          else
          {
            // Over Possession Limit
          }
        }

        // Don't take up CPU resources
        pros::delay(100);
      }
    },
    "POSSESSION TASK");

// Auto Sort thread
pros::Task autoSortTask(
    []() {
      while (true)
      {
        if (autoSortEnabled)  // If enabled
        {
          // Check if the ring is detected
          RingChangeDetector.check(ringDetector.get() < 50);

          // If ring detection has changed from none, to ring
          if (RingChangeDetector.getChanged() && RingChangeDetector.getValue())
          {
            // Check team color (Convert to ring state)
            int teamColor = Screen::isRedTeam() ? 1 : 2;

            // If ring current ring isn't ring color, and there are rings
            if (possession[0] != teamColor && possession[0] != ringState::none)
            {
              // Disable the intake (stop if from spinning)
              ringSort.extend();   // Eject the ring
              pros::delay(400);    // Wait for Pneumatic to extend
              ringSort.retract();  // Retract the intake
            }
            possession[0] = possession[1];    // Ring 2 is now Ring 1
            possession[1] = ringState::none;  // Remove ring from possession
          }
          // Don't take up CPU resources
          pros::delay(50);
        }
      }
    },
    "AUTO-SORT TASK");

}  // namespace Intake
}  // namespace Systems