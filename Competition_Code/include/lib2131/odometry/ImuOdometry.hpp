#pragma once

#include "lib2131/odometry/AbstractOdometry.hpp"
#include "lib2131/utilities/Point.hpp"
#include "lib2131/utilities/Units.h"
#include "pros/imu.hpp"

namespace lib2131::odometry
{

class ImuOdometry : public AbstractOdometry
{
  using angle_t = units::angle::radian_t;
  using distance_t = units::length::inch_t;

 private:  // Variables
  std::shared_ptr<pros::Imu> m_pIMU;
  utilities::Point m_DeltaXY;

 public:  // Constructors
  ImuOdometry(const std::shared_ptr<pros::Imu>& IMU)
      : AbstractOdometry(),
        m_pIMU(std::move(IMU)),
        m_DeltaXY(distance_t(0), distance_t(0))
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

    // Inertial Angle
    this->m_deltaTheta =
        this->m_currentTheta - units::angle::degree_t(m_pIMU->get_heading() * -1);

    angle_t avgTheta = this->m_currentTheta - this->m_deltaTheta / 2;
    this->m_currentTheta -= this->m_deltaTheta;

    // Get Acceleration in G's -> Convert to In/s -> Store as Point
    auto IMUAccel = this->m_pIMU->get_accel();
    this->m_DeltaXY += utilities::Point(distance_t(IMUAccel.x / 386.0885826772),
                                        distance_t(IMUAccel.y / 386.0885826772));

    // Update Position
    this->updateStates(utilities::Pose(m_DeltaXY, this->m_deltaTheta), avgTheta,
                       deltaTime);
  }

  void calibrate() override { this->m_pIMU->reset(true); }
};
}  // namespace lib2131::odometry
