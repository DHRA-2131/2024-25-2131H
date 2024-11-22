/**
 * @file gps-odometry.hpp
 * @author Andrew Hilton (2131H)
 * @brief Class declaration for GPS Localization
 * @version 0.1
 * @date 2024-11-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "2131H/Systems/Odometry/abstract-odometry.hpp"
#include "2131H/Utilities/pose.hpp"
#include "pros/gps.hpp"

namespace Systems
{
class GpsOdometry : AbstractOdometry
{
 private:  // === Member Variables  === //
  std::vector<pros::v5::Gps*> sensors;

 public:  // === Public Variables === //
  ;

 public:  // === Getters / Setters === //
  Pose getGlobalPose()
  {
    Pose output(0, 0, 0);
    for (auto gps : sensors)
    {
      // if (gps.get_error())
      output = output + Pose(gps->get_position_x(), gps->get_position_y(), gps->get_heading());
    }
  }
  Pose updatePose() {}
  Pose localPose() {}

 public:  // === Constructors === //
  ;

 private:  // === Member Functions === //
  ;

 public:  // === Functions === //
  ;
};
}  // namespace Systems