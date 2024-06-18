#pragma once
#include <memory>

#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "lib2131/utilities/Pose.hpp"

namespace lib2131::odometry
{
using namespace trackingWheel;

class WheelOdometry : public AbstractOdometry
{
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
        m_viable(leftWheel->getOffset() - rightWheel->getOffset() != 0)
  {
  }

  WheelOdometry(const std::shared_ptr<AbstractTrackingWheel>& leftWheel,
                const std::shared_ptr<AbstractTrackingWheel>& rightWheel)
      : AbstractOdometry(),
        m_pLeftWheel(leftWheel),
        m_pRightWheel(rightWheel),
        m_hasRear(false),
        m_viable(leftWheel->getOffset() - rightWheel->getOffset() != 0)
  {
  }

 public:  // Functions
  void update(double deltaTime) override
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
    this->m_deltaTheta = utilities::Angle(
        (m_pLeftWheel->getDeltaDistance() - m_pRightWheel->getDeltaDistance()) /
            (m_pLeftWheel->getOffset() - m_pRightWheel->getOffset()),
        false);

    utilities::Angle avgTheta = this->m_currentTheta - this->m_deltaTheta / 2;
    this->m_currentTheta -= this->m_deltaTheta;

    // Calculate Local Change
    double localDeltaX = this->calculateChordLength(
        m_pLeftWheel->getDeltaDistance(), m_pLeftWheel->getOffset(), this->m_deltaTheta);

    double localDeltaY = 0;
    if (m_hasRear)
    {
      localDeltaY =
          this->calculateChordLength(m_pRearWheel->getDeltaDistance(),
                                     m_pRearWheel->getOffset(), this->m_deltaTheta);
    }

    this->updateStates(
        utilities::Pose(localDeltaX, localDeltaY, this->m_deltaTheta).rotate(avgTheta),
        deltaTime);
  }
  void calibrate() override
  {
    m_pLeftWheel->reset();
    m_pRightWheel->reset();
    if (this->m_hasRear) m_pRearWheel->reset();
  }
};
}  // namespace lib2131::odometry
