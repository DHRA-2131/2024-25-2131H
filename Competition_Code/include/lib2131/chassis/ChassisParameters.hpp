#pragma once

#include "lib2131/utilities/Units.h"

namespace lib2131::chassis
{

struct ChassisParameters
{
  using angular_acceleration_t =
      units::unit_t<units::angular_velocity::radians_per_second,
                    units::inverse<units::time::seconds>>;

  // Required
  units::length::inch_t wheelDiameter;
  units::length::inch_t trackWidth;
  units::angular_velocity::revolutions_per_minute_t maxWheelRpm;
  units::acceleration::feet_per_second_squared_t maxLinearAcceleration;
  angular_acceleration_t maxAngularAcceleration;

  //* Calculated (Unless Specified)
  // (Inches * PI * Revolutions) / (Minutes * 60) = Feet / Second
  units::velocity::feet_per_second_t maxLinearVelocity =
      units::velocity::feet_per_second_t(wheelDiameter.value() * maxWheelRpm.value() /
                                         60 * units::constants::pi.value());

  // Assume Decel is equal Accel
  units::acceleration::feet_per_second_squared_t maxLinearDeceleration =
      maxLinearAcceleration;

  // Radians / Second = Linear Velocity / Radius (Diameter / 2)
  units::angular_velocity::radians_per_second_t maxAngularVelocity =
      units::angular_velocity::radians_per_second_t(2 * maxLinearVelocity.value() /
                                                    trackWidth.value());
  // Assume Decel is equal to Accel
  angular_acceleration_t maxAngularDeceleration = maxAngularAcceleration;
};

}  // namespace lib2131::chassis