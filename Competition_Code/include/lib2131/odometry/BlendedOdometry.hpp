#pragma once
#include <memory>

#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "lib2131/utilities/Units.h"
#include "pros/imu.hpp"

namespace lib2131::odometry
{
using namespace trackingWheel;

class BlendedOdometry : public AbstractOdometry
{
  using angle_t = units::angle::radian_t;
  using distance_t = units::length::inch_t;

 private:  // Variables
  std::shared_ptr<AbstractTrackingWheel> m_pVerticalWheel;
  std::shared_ptr<AbstractTrackingWheel> m_pHorizontalWheel;
  std::shared_ptr<pros::Imu> m_pIMU;
  const bool m_hasHorizontal;

 public:  // Constructors
  BlendedOdometry(const std::shared_ptr<AbstractTrackingWheel>& verticalWheel,
                  const std::shared_ptr<AbstractTrackingWheel>& horizontalWheel,
                  const std::shared_ptr<pros::Imu>& IMU)
      : AbstractOdometry(),
        m_pVerticalWheel(std::move(verticalWheel)),
        m_pHorizontalWheel(std::move(horizontalWheel)),
        m_hasHorizontal(true),
        m_pIMU(std::move(IMU))
  {
  }

  BlendedOdometry(const std::shared_ptr<AbstractTrackingWheel>& verticalWheel,
                  const std::shared_ptr<pros::Imu>& IMU)
      : AbstractOdometry(),
        m_pVerticalWheel(std::move(verticalWheel)),
        m_hasHorizontal(false),
        m_pIMU(std::move(IMU))
  {
  }

 public:  // Functions
  void update(units::time::millisecond_t deltaTime) override
  {
    if (std::isinf(m_pIMU->get_heading()))
    {
      // Wait For Inertial to have a value
      return;
    }

    // Update Wheels
    m_pVerticalWheel->update();
    if (m_hasHorizontal) m_pHorizontalWheel->update();

    // Inertial Angle
    this->m_deltaTheta = this->m_currentTheta - angle_t(m_pIMU->get_heading() * -1);

    angle_t avgTheta = this->m_currentTheta - this->m_deltaTheta / 2;
    this->m_currentTheta -= this->m_deltaTheta;

    // Calculate Local Change
    distance_t localDeltaX =
        this->calculateChordLength(m_pVerticalWheel->getDeltaDistance(),
                                   m_pVerticalWheel->getOffset(), this->m_deltaTheta);
    distance_t localDeltaY(0);
    if (m_hasHorizontal)
    {
      localDeltaY =
          this->calculateChordLength(m_pHorizontalWheel->getDeltaDistance(),
                                     m_pHorizontalWheel->getOffset(), this->m_deltaTheta);
    }

    this->updateStates(utilities::Pose({localDeltaX, localDeltaY}, this->m_deltaTheta),
                       avgTheta, deltaTime);
  }

  void calibrate() override
  {
    this->m_pIMU->reset(true);
    this->m_pVerticalWheel->reset();
    if (this->m_hasHorizontal) this->m_pHorizontalWheel->reset();
  }
};
}  // namespace lib2131::odometry
