#pragma once
#include "Units.h"

namespace units
{
// Inches Per Second
UNIT_ADD(velocity, inch_per_second, inches_per_second, inps,
         units::unit<std::ratio<1, 12>, units::velocity::feet_per_second>)
// Tiles
UNIT_ADD(length, tile, tiles, tile, units::unit<std::ratio<2, 1>, units::length::feet>)
}  // namespace units
