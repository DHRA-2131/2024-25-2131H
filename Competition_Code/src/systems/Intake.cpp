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
struct ringColors
{
  static const int none = 0;
  static const int red = 1;
  static const int blue = 2;
};

int possession[2] = {0, 0};
ChangeDetector<int> PossessionChangeDetector(ringColors::none);
ChangeDetector<bool> RingChangeDetector;

void teleOp()
{
  if (enabled)
  {
    if (Buttons::Intake.isPressing()) { motor.move_voltage(12000); }
    else if (Buttons::Outtake.isPressing()) { motor.move_voltage(-12000); }
    else { motor.brake(); }
  }
}

pros::Task possessionTask(
    []() {
      colorDetector.set_led_pwm(100);
      while (true)
      {
        auto color = colorDetector.get_hue();
        if (color < 20 && color != 0) { PossessionChangeDetector.check(ringColors::red); }
        else if (color > 100) { PossessionChangeDetector.check(ringColors::blue); }
        else { PossessionChangeDetector.check(ringColors::none); }

        auto ring = PossessionChangeDetector.getValue();

        if (PossessionChangeDetector.getChanged() && (ring == ringColors::red || ring == ringColors::blue) &&
            motor.get_actual_velocity() > 10)
        {
          if (possession[0] == 0) { possession[0] = ring; }
          else if (possession[1] == 0) { possession[1] = ring; }
          else
          {
            // Over Possession Limit
          }
        }

        pros::delay(100);
      }
    },
    "POSSESSION TASK");

pros::Task autoSortTask(
    []() {
      while (true)
      {
        RingChangeDetector.check(ringDetector.get() < 20);
        if (RingChangeDetector.getChanged() && RingChangeDetector.getValue())
        {
          int teamColor = Screen::isRedTeam() ? 1 : 2;
          if (possession[0] != teamColor && possession[0] != ringColors::none)
          {
            enabled = false;
            pros::delay(65);
            motor.move_voltage(1000);
            pros::delay(100);
            enabled = true;
          }
          possession[0] = possession[1];
          possession[1] = ringColors::none;
        }
        pros::delay(50);
      }
    },
    "AUTO-SORT TASK");

}  // namespace Intake
}  // namespace Systems