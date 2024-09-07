#include "main/Screen.hpp"

#include "ChangeDetector.hpp"
#include "RobotConfig.hpp"
#include "pros/misc.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
namespace Screen
{
void NULL_AUTON() {}
std::vector<AutonCard> Cards = {{"LEFT", "Left Description", NULL_AUTON},
                                {"RIGHT", "Right Description", NULL_AUTON},
                                {"SKILLS", "Skills Description", NULL_AUTON},
                                {"DEBUG", "Debug Description", NULL_AUTON}};

bool debug(true);
bool initial(true);
int index = -1;
ChangeDetector<bool> ScreenDetector;

void update()
{
  ScreenDetector.check(pros::screen::touch_status().touch_status ==
                       pros::E_TOUCH_PRESSED);

  if (ScreenDetector.getChanged() && ScreenDetector.getValue() || initial)
  {
    index++;
    if (index >= Cards.size()) { index = 0; }
    Cards[index].draw();
  }

  if (debug)
  {
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN,
                        HEIGHT - MEDIUM_TEXT_MARGIN - MEDIUM_TEXT_SIZE,
                        "X: %f, Y: %f, Theta: %f", pose.x, pose.y, pose.theta);
    pros::screen::print(pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN,
                        HEIGHT - MEDIUM_TEXT_MARGIN - 2 * MEDIUM_TEXT_SIZE,
                        "Robot Battery %: %f", pros::battery::get_capacity());
  }

  initial = false;
}
}  // namespace Screen