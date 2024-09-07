#pragma once

#include <initializer_list>
#include <string>

#include "pros/screen.h"
#include "pros/screen.hpp"

namespace Screen
{
extern bool debug;

#define WIDTH 480
#define HEIGHT 240
#define PEN_WIDTH 1

#define LARGE_TEXT_MARGIN 10
#define LARGE_TEXT_SIZE 30

#define MEDIUM_TEXT_MARGIN 10
#define MEDIUM_TEXT_SIZE 20

void update();

class AutonCard
{
 private:  // Variables
  std::string m_name;
  std::string m_setupDesc;

  void (*m_pAutonCallback)(void);

 public:  // Constructors
  AutonCard(std::string name, std::string setupDesc, void (*AutonCallback)(void))
      : m_name(name), m_setupDesc(setupDesc), m_pAutonCallback(AutonCallback)
  {
  }

 public:  // Methods
  void draw()
  {
    pros::screen::set_eraser(0x0a1400);             // Brown
    pros::screen::set_pen(RGB2COLOR(224, 114, 0));  // Gold

    // Clear Screen
    pros::screen::erase_rect(0, 0, WIDTH, HEIGHT);

    // Draw Background
    pros::screen::draw_rect(PEN_WIDTH, PEN_WIDTH, WIDTH - PEN_WIDTH, HEIGHT - PEN_WIDTH);
    pros::screen::draw_rect(PEN_WIDTH, PEN_WIDTH, WIDTH - PEN_WIDTH,
                            2 * LARGE_TEXT_MARGIN + LARGE_TEXT_SIZE + PEN_WIDTH);

    // Display Name
    pros::screen::print(pros::E_TEXT_LARGE, LARGE_TEXT_MARGIN, LARGE_TEXT_MARGIN,
                        m_name.c_str());
    pros::screen::print(
        pros::E_TEXT_MEDIUM, MEDIUM_TEXT_MARGIN,
        2 * LARGE_TEXT_MARGIN + LARGE_TEXT_SIZE + PEN_WIDTH + MEDIUM_TEXT_MARGIN,
        m_setupDesc.c_str());
  }
};
}  // namespace Screen