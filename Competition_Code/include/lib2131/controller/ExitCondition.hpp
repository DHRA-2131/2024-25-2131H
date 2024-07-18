#pragma once
#include "lib2131/utilities/Pose.hpp"
#include "lib2131/utilities/Units.h"

namespace lib2131::controller::exit_condition
{
using namespace units::literals;

// TODO: UPDATE STRUCT DEFAULTS
//? ESTIMATES ARE CURRENTLY JUST GUESSES
struct settleParam
{
  units::time::second_t largeErrorTime = 700_ms;
  units::time::second_t smallErrorTime = 300_ms;
  units::time::second_t thruErrorTime = 0_ms;
};

struct distanceParam
{
  units::length::inch_t smallErrorDistance = 3_in;
  units::length::inch_t largeErrorDistance = 5_in;
  units::length::inch_t thruDistance = 4_in;
};

struct angleParam
{
  units::angle::degree_t smallErrorDistance = 2_deg;
  units::angle::degree_t largeErrorDistance = 4_deg;
  units::angle::degree_t thruDistance = 10_deg;
};

class ExitCondition
{
 private:  // Variables
  units::time::second_t settleTimer;
  settleParam m_settleConditions;
  distanceParam m_distanceConditions;
  angleParam m_angleConditions;
  utilities::Pose m_target;

 public:  // Constructors
  ExitCondition()
      : m_settleConditions(settleParam()), m_distanceConditions(distanceParam())
  {
  }
  ExitCondition(settleParam settleConditions, distanceParam distanceConditions,
                angleParam angleConditions)
      : m_settleConditions(settleConditions),
        m_distanceConditions(distanceConditions),
        m_angleConditions(angleConditions)
  {
  }

 public:  // Methods
  static ExitCondition newBuilder() { return ExitCondition(); }

  void setTarget(utilities::Pose newTarget) { m_target = newTarget; }

  // Add / Set Exit Conditional
  void setSettle(settleParam newSettleConditions)
  {
    m_settleConditions = newSettleConditions;
  }

  void setExitDistances(distanceParam newDistanceConditions)
  {
    m_distanceConditions = newDistanceConditions;
  }

  bool canExit(utilities::Pose testedPose, bool thru, bool checkAngle,
               units::time::second_t deltaTime)
  {
    settleTimer += deltaTime;  // Update Timer
    //* Note: Error is unsigned
    units::length::inch_t linearError = testedPose.pos.magnitude(m_target.pos);
    units::angle::degree_t angleError =
        units::math::fabs(m_target.theta - testedPose.theta);

    if (thru && linearError < m_distanceConditions.thruDistance &&
        settleTimer > m_settleConditions.thruErrorTime)
    {
      return true;  // Meets Thru Error Conditions
    }
    else if (linearError < m_distanceConditions.largeErrorDistance &&
             settleTimer > m_settleConditions.largeErrorTime)
    {
      return true;  // Meets Large Error Conditions
    }
    else if (linearError < m_distanceConditions.smallErrorDistance &&
             settleTimer < m_settleConditions.smallErrorTime)
    {
      return true;  // Meets Small Error Conditions
    }
    else if (linearError > m_distanceConditions.largeErrorDistance &&
             linearError > m_distanceConditions.smallErrorDistance &&
             linearError > m_distanceConditions.thruDistance)
    {
      settleTimer = 0_s;  // Outside of settle range
      return false;       // Cannot Exit
    }
    else
    {
      return false;  // Hasn't met settle Time
    }
  }
};
}  // namespace lib2131::controller::exit_condition