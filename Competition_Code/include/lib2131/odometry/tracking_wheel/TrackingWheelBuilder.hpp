#pragma once
#include <cstdint>
#include <memory>

#include "lib2131/odometry/tracking_wheel/ADITrackingWheel.hpp"
#include "lib2131/odometry/tracking_wheel/MotorTrackingWheel.hpp"
#include "lib2131/odometry/tracking_wheel/RotationalTrackingWheel.hpp"
#include "lib2131/utilities/Units.h"
#include "pros/motor_group.hpp"

namespace lib2131::odometry::trackingWheel
{

using namespace units::literals;

class TrackingWheelBuilder
{
  using distance_t = units::length::inch_t;

 private:  // Tracking Wheel Configuration
  distance_t m_offset;
  distance_t m_wheelDiameter;
  double m_ratio;
  distance_t m_distance;

 public:  // Constructors / Deconstructor
  TrackingWheelBuilder() : m_offset(0), m_wheelDiameter(0), m_ratio(1), m_distance(0) {}
  ~TrackingWheelBuilder() {}

 public:  // Configuration
  TrackingWheelBuilder& setOffset(distance_t offset)
  {
    m_offset = offset;
    return *this;
  }
  TrackingWheelBuilder& setWheelDiameter(distance_t wheelDiameter)
  {
    m_wheelDiameter = wheelDiameter;
    return *this;
  }
  TrackingWheelBuilder& setRatio(float ratio)
  {
    m_ratio = ratio;
    return *this;
  }
  TrackingWheelBuilder& setInitalDistance(distance_t distance)
  {
    m_distance = distance;
    return *this;
  }

 public:  // Build / Sensor
  static TrackingWheelBuilder newBuilder() { return TrackingWheelBuilder(); }

  std::shared_ptr<ADITrackingWheel> buildADIEncoderTrackingWheel(int8_t port)
  {
    // Check if necessary info is included
    if (m_ratio == 0 || m_offset == 0_in || m_wheelDiameter == 0_in)
    {
      throw "Build Tracking Wheel Failed";
      return nullptr;
    }
    return std::make_shared<ADITrackingWheel>(port, m_offset, m_wheelDiameter, m_ratio,
                                              m_distance);
  }
  std::shared_ptr<RotationalTrackingWheel> buildRotationalTrackingWheel(int8_t port)
  {
    // Check if necessary info is included
    if (m_ratio == 0 || m_offset == 0_in || m_wheelDiameter == 0_in)
    {
      throw "Build Tracking Wheel Failed";
      return nullptr;
    }
    return std::make_shared<RotationalTrackingWheel>(port, m_offset, m_wheelDiameter,
                                                     m_ratio, m_distance);
  }
  std::shared_ptr<MotorTrackingWheel> buildMotorTrackingWheel(
      const std::initializer_list<std::int8_t> ports, double rpm)
  {
    // Check if necessary info is included
    if (m_ratio == 0 || m_offset == 0_in || m_wheelDiameter == 0_in)
    {
      throw "Build Tracking Wheel Failed";
      return nullptr;
    }
    return std::make_shared<MotorTrackingWheel>(std::make_shared<pros::MotorGroup>(ports),
                                                m_offset, m_wheelDiameter, rpm,
                                                m_distance);
  }
};

}  // namespace lib2131::odometry::trackingWheel
