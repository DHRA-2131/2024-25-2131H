/**
 * @file abstract-odometry.hpp
 * @author Andrew Hilton (2131H)
 * @brief Class Declaration for the Abstract Odometry Class
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "2131H/Utilities/pose.hpp"

using namespace Utilities;

namespace Systems
{
class AbstractOdometry
{
 public:  // === Functions === //
  /**
   * @brief Updates the Position of a pose using a localization method
   * @note Is abstract as to allow multiple kinds of Odometry.
   * @param currentPose Current Position
   * @return Pose Updated Position
   */
  virtual Pose updatePose(Pose& currentPose) = 0;
  /**
   * @brief Initializes all the sensors needed for odometry to function
   * @param blocking Wait for sensors to calibrate?
   */
  virtual void init(bool blocking = true) = 0;
};
}  // namespace Systems