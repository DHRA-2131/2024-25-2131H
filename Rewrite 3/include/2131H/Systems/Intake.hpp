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
  std::shared_ptr<pros::v5::Motor> m_pFirstMotor;
  std::shared_ptr<pros::v5::Motor> m_pSecondMotor;
  std::shared_ptr<pros::v5::Optical> m_pOptical;
  std::shared_ptr<pros::adi::Pneumatics> m_pLift;

  std::int32_t m_sortDistance;
  double m_redBound;
  double m_blueBound;

  bool m_sortEnabled;
  RingColors m_sortColor;
  bool m_sorted = true;

  std::vector<RingColors> m_possession = {};
  double m_position = 0;
  double m_sortPosition = 0;

  ChangeDetector<RingColors> m_colorStateDetector;

  Utilities::ButtonDetector intakeButton;
  Utilities::ButtonDetector outtakeButton;
  Utilities::ButtonDetector liftButton;

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
      pros::Motor* pFirstMotor,
      pros::Motor* pSecondMotor,
      pros::Optical* pOptical,
      pros::adi::Pneumatics* pLift,
      double sortDistance,
      pros::controller_digital_e_t intakeBtn,
      pros::controller_digital_e_t outtakeBtn,
      pros::controller_digital_e_t liftBtn,
      pros::Controller* pController,
      double redBound = 30,
      double blueBound = 120);

 protected:  // *** ==== Protected Methods ==== *** //
  void _update();

 public:  // *** ==== Public Methods ==== *** //
  void spin(double voltage = 12000.0, uint stage = 2);
  void stop();
  void teleOp();
  void init();
  void lift();
  void drop();
  void toggleLift();
  bool isLifted();
};