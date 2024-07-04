#pragma once
#include <memory>

#include "BlendedOdometry.hpp"
#include "WheelOdometry.hpp"
#include "lib2131/odometry/tracking_wheel/AbstractTrackingWheel.hpp"
#include "pros/imu.hpp"
#include "tracking_wheel/AbstractTrackingWheel.hpp"

namespace lib2131::odometry
{
enum class WheelLocation
{

  Left = 0,
  Right = 1,
  Rear = 2,

  Vert1 = 0,
  Vert2 = 1,
  Horz = 2,
};

using namespace trackingWheel;
class OdometryBuilder
{
 private:  // Variables
  std::shared_ptr<AbstractTrackingWheel> m_pLeftWheel;
  std::shared_ptr<AbstractTrackingWheel> m_pRightWheel;
  std::shared_ptr<AbstractTrackingWheel> m_pRearWheel;
  std::shared_ptr<pros::Imu> m_pIMU;

 public:  // Constructors
  OdometryBuilder() : m_pLeftWheel(), m_pRightWheel(), m_pRearWheel(), m_pIMU() {}
  static OdometryBuilder newBuilder() { return OdometryBuilder(); }

 public:  // Add Sensors
  OdometryBuilder& addTrackingWheel(std::shared_ptr<AbstractTrackingWheel> TrackingWheel,
                                    WheelLocation Location)
  {
    switch (Location)
    {
      case WheelLocation::Left:
        m_pLeftWheel = std::move(TrackingWheel);
        break;
      case WheelLocation::Right:
        m_pRightWheel = std::move(TrackingWheel);
        break;
      case WheelLocation::Rear:
        m_pRearWheel = std::move(TrackingWheel);
        break;
      default:
        throw "Not a Wheel Location";
        break;
    }
    return *this;
  }
  OdometryBuilder& addInertialUnit(pros::Imu& IMU)
  {
    m_pIMU = std::make_shared<pros::Imu>(IMU);
    return *this;
  }

 public:  // Build
  // TODO : Make one builder
  std::shared_ptr<BlendedOdometry> buildBlendedOdometry()
  {
    if (m_pLeftWheel != nullptr && m_pIMU != nullptr)
    {
      if (m_pRearWheel != nullptr)
      {
        return std::make_shared<BlendedOdometry>(
            BlendedOdometry(m_pLeftWheel, m_pRearWheel, m_pIMU));
      }
      else
      {
        return std::make_shared<BlendedOdometry>(BlendedOdometry(m_pLeftWheel, m_pIMU));
      }
    }
    else
    {
      throw "Couldn't Build Odometry";
      return nullptr;
    }
  }

  std::shared_ptr<WheelOdometry> buildWheelOdometry()
  {
    if (m_pLeftWheel != nullptr && m_pRightWheel != nullptr)
    {
      if (m_pRearWheel != nullptr)
      {
        return std::make_shared<WheelOdometry>(
            WheelOdometry(m_pLeftWheel, m_pRightWheel, m_pRearWheel));
      }
      else
      {
        return std::make_shared<WheelOdometry>(
            WheelOdometry(m_pLeftWheel, m_pRightWheel));
      }
    }
    else
    {
      throw "Couldn't Build Odometry";
      return nullptr;
    }
  }

  // TODO: ADD IMU ODOMETRY
};
}  // namespace lib2131::odometry
