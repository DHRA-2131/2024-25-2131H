/**
 * @file Intake.hpp
 * @author Andrew Hilton (2131H)
 * @brief Intake Class Declaration
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include <cstdint>
#include <memory>
#include <vector>

#include "2131H/Utilities/ButtonDetector.hpp"
#include "2131H/Utilities/ChangeDetector.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"

class Intake
{
 public:  // *** ==== Enums ==== *** //
  enum class RingColors
  {
    NONE,
    RED,
    BLUE,
  };

 private:  // *** ==== Member Variables ==== *** //
  std::shared_ptr<pros::v5::MotorGroup> m_pMotors;
  std::shared_ptr<pros::v5::Optical> m_pOptical;
  std::shared_ptr<pros::v5::Distance> m_pDistance;
  std::shared_ptr<pros::adi::Pneumatics> m_pPneumatics;

  std::int32_t m_sortDistance;
  double m_redBound;
  double m_blueBound;

  bool m_sortEnabled;
  RingColors m_sortColor;
  std::vector<RingColors> m_possession = {};

  ChangeDetector<RingColors> m_colorStateDetector;
  ChangeDetector<bool> m_ringStateDetector;

  Utilities::ButtonDetector intakeButton;
  Utilities::ButtonDetector outtakeButton;
  pros::Task m_thread;
  bool m_initialized = false;

 public:  // *** ==== Getters / Setters ==== *** //
  void enableSort(RingColors sortColor);
  void disableSort();

  RingColors getSortColor();
  RingColors getCurrentRingColor();
  double getPossessionCount();
  std::vector<RingColors> getPossession();

 public:  // *** ==== Constructors / Deconstructors ==== *** //
  Intake(
      pros::MotorGroup* pMotors,
      pros::Optical* pOptical,
      pros::Distance* pDistance,
      pros::adi::Pneumatics* pPneumatics,
      std::int32_t sortDistance,
      pros::controller_digital_e_t intakeBtn,
      pros::controller_digital_e_t outtakeBtn,
      pros::Controller* pController,
      double redBound = 30,
      double blueBound = 120);

 protected:  // *** ==== Protected Methods ==== *** //
  void _update();

 public:  // *** ==== Public Methods ==== *** //
  void spin(double voltage = 12000.0);

  void stop();

  void teleOp();
  void init();
};