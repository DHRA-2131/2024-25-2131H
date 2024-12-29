#pragma once
#include "2131H/Systems/Arm.hpp"
#include "2131H/Systems/Clamp.hpp"
#include "2131H/Systems/Intake.hpp"
#include "2131H/Systems/Other.hpp"
#include "2131H/Utilities/Console.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"

extern Arm arm;
extern Intake intake;
extern Clamp clamp;
extern Doinkler doinkler;
extern RingRush rush;
extern Terminal Console;

extern pros::adi::DigitalIn teamColor;
extern pros::adi::DigitalIn cycleAuton;

extern pros::Controller primary;
extern lemlib::Chassis chassis;
