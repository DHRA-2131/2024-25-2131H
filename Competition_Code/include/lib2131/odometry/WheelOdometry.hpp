#pragma once
#include <memory>

#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "lib2131/utilities/Pose.hpp"
#include "lib2131/utilities/Units.h"

namespace lib2131::odometry
{
using namespace trackingWheel;

class WheelOdometry : public AbstractOdometry
{
  using distance_t = units::length::inch_t;
  using angle_t = units::angle::radian_t;

 private:  // Variables
  std::shared_ptr<AbstractTrackingWheel> m_pLeftWheel;
  std::shared_ptr<AbstractTrackingWheel> m_pRightWheel;
  std::shared_ptr<AbstractTrackingWheel> m_pRearWheel;

  const bool m_hasRear;
  const bool m_viable;

 public:  // Constructors
  WheelOdometry(const std::shared_ptr<AbstractTrackingWheel>& leftWheel,
                const std::shared_ptr<AbstractTrackingWheel>& rightWheel,
                const std::shared_ptr<AbstractTrackingWheel>& rearWheel)
      : AbstractOdometry(),
        m_pLeftWheel(std::move(leftWheel)),
        m_pRightWheel(std::move(rightWheel)),
        m_pRearWheel(std::move(rearWheel)),
        m_hasRear(true),
        m_viable(leftWheel->getOffset() - rightWheel->getOffset() != distance_t(0))
  {
  }

  WheelOdometry(const std::shared_ptr<AbstractTrackingWheel>& leftWheel,
                const std::shared_ptr<AbstractTrackingWheel>& rightWheel)
      : AbstractOdometry(),
        m_pLeftWheel(leftWheel),
        m_pRightWheel(rightWheel),
        m_hasRear(false),
        m_viable(leftWheel->getOffset() - rightWheel->getOffset() != distance_t(0))
  {
  }

 public:  // Functions
  void update(units::time::millisecond_t deltaTime) override
  {
    if (!m_viable)
    {
      throw "Not a Viable Odometry Configuration";
      return;
    }

    m_pLeftWheel->update();
    m_pRightWheel->update();
    if (m_hasRear) m_pRearWheel->update();

    // Inertial Angle
    this->m_deltaTheta =
        angle_t((m_pLeftWheel->getDeltaDistance() - m_pRightWheel->getDeltaDistance()) /
                    (m_pLeftWheel->getOffset() - m_pRightWheel->getOffset()),
                false);

    angle_t avgTheta = this->m_currentTheta - this->m_deltaTheta / 2;
    this->m_currentTheta -= this->m_deltaTheta;

    // Calculate Local Change
    distance_t localDeltaX = this->calculateChordLength(
        m_pLeftWheel->getDeltaDistance(), m_pLeftWheel->getOffset(), this->m_deltaTheta);

    distance_t localDeltaY(0);
    if (m_hasRear)
    {
      localDeltaY =
          this->calculateChordLength(m_pRearWheel->getDeltaDistance(),
                                     m_pRearWheel->getOffset(), this->m_deltaTheta);
    }

    this->updateStates(utilities::Pose({localDeltaX, localDeltaY}, this->m_deltaTheta),
                       avgTheta, deltaTime);
  }
  void calibrate() override
  {
    m_pLeftWheel->reset();
    m_pRightWheel->reset();
    if (this->m_hasRear) m_pRearWheel->reset();
  }
};
}  // namespace lib2131::odometry
