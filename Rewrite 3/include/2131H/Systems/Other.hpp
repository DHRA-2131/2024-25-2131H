/**
 * @file Other.hpp
 * @author Andrew Hilton (2131H)
 * @brief Other systems such as Doinkler
 * @version 0.1
 * @date 2024-12-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include <memory>

#include "2131H/Utilities/ButtonDetector.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

class Doinkler : public pros::adi::Pneumatics
{
 private:                                       // *** ==== Member Variables ==== *** //
  Utilities::ButtonDetector toggleBtnDetector;  // - State checker for Controller Behavior

 public:  // *** ==== Public Constructor ==== *** //
  Doinkler(
      std::uint8_t port,
      bool startExtended,
      pros::controller_digital_e_t button,
      pros::Controller* controller);

 public:  // *** ==== Public Methods ==== *** //
  void teleOp();
};

class RingRush
{
 private:                                       // *** ==== Member Variables ==== *** //
  Utilities::ButtonDetector toggleBtnDetector;  // - State checker for Controller Behavior
  std::shared_ptr<pros::adi::Pneumatics> m_pPneumatic1;
  std::shared_ptr<pros::adi::Pneumatics> m_pPneumatic2;

 public:  // *** ==== Public Constructor ==== *** //
  RingRush(
      std::uint8_t port1,
      std::uint8_t port2,
      pros::controller_digital_e_t button,
      pros::Controller* controller,
      bool startExtended = false);

 public:  // *** ==== Public Methods ==== *** //
  void teleOp();
  void extend();
  void retract();
};