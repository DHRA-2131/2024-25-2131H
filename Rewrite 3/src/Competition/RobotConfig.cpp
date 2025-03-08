/**
 * @file RobotConfig.cpp
 * @author Andrew Hilton (2131H)
 * @brief Robot Configuration File
 * @version 0.1
 * @date 2024-12-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Competition/RobotConfig.hpp"

#include "2131H/Systems/Clamp.hpp"
#include "2131H/Systems/Intake.hpp"
#include "2131H/Utilities/Console.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

/**
 * -----------------------------------------------------------------------------------------
 * *          _+=+_
 * *       .-`  .  `-.          8888888b.  8888888b.   .d88888b.   .d8888b.
 * *    _+`     "     `+_       888   Y88b 888   Y88b d88P" "Y88b d88P  Y88b
 * *   \\\sssssssssssss///      888    888 888    888 888     888 Y88b.
 * *      .ss\  *  /ss.         888   d88P 888   d88P 888     888  "Y888b.
 * *  .+bm  .s  *  s.  md+.     8888888P"  8888888P"  888     888     "Y88b.
 * * .hMMMMs .  *  . sMMMMh.    888        888 T88b   888     888       "888
 * *  `\hMMMb \ | / dMMMh:      888        888  T88b  Y88b. .d88P Y88b  d88P
 * *    -SNMNo  -  oNMNs-       888        888   T88b  "Y88888P"   "Y8888P"
 * *      `+dMh\./dMd/
 * *         `:yNy:`                      Powered by PROS for VEX V5
 * *            "
 * -------------------------------------------------------------------------------------------
 */

/** // ROBOT 6 Ports
    // LD -5 -6 -7
    // RD 8 9 10
    // Inertial 21
    // Intake 1 4
    // In Op 11
    // In Dist 13
    // In Sort G
    // In Lift F
    // Arm 2
    // Arm Rot -3
    // Clamp Pneu H
    // Clamp Dist 19
    // Team Cycle D
    // Cycle Auto E
*/

pros::MotorGroup leftDrive(
    {-15, -16, -20}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
pros::MotorGroup rightDrive(
    {17, 18, 19}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Imu inertial(21);

namespace IntakeConfig
{
pros::Motor first(-14, pros::MotorGears::green, pros::v5::MotorEncoderUnits::deg);
pros::Motor second(-9, pros::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);

pros::Optical optical(1);
pros::adi::Pneumatics lift('F', false, false);
}  // namespace IntakeConfig

namespace ArmConfig
{
pros::MotorGroup motors({12}, pros::MotorGears::green, pros::v5::MotorEncoderUnits::deg);
pros::Rotation rotation(-11);
}  // namespace ArmConfig

namespace ClampConfig
{
pros::adi::Pneumatics pneumatic('A', false, false);
pros::Distance distance(10);
}  // namespace ClampConfig

pros::Controller primary(pros::E_CONTROLLER_MASTER);

pros::adi::DigitalIn teamColor('G');
pros::adi::DigitalIn cycleAuton('H');

/**
 * ------------------------------------------------------------------------------------------
 *!  222222222222222      1111111    333333333333333     1111111   HHHHHHHHH     HHHHHHHHH
 *! 2:::::::::::::::22   1::::::1   3:::::::::::::::33  1::::::1   H:::::::H     H:::::::H
 *! 2::::::222222:::::2 1:::::::1   3::::::33333::::::31:::::::1   H:::::::H     H:::::::H
 *! 2222222     2:::::2 111:::::1   3333333     3:::::3111:::::1   HH::::::H     H::::::HH
 *!             2:::::2    1::::1               3:::::3   1::::1     H:::::H     H:::::H
 *!             2:::::2    1::::1               3:::::3   1::::1     H:::::H     H:::::H
 *!          2222::::2     1::::1       33333333:::::3    1::::1     H::::::HHHHH::::::H
 *!     22222::::::22      1::::l       3:::::::::::3     1::::l     H:::::::::::::::::H
 *!   22::::::::222        1::::l       33333333:::::3    1::::l     H:::::::::::::::::H
 *!  2:::::22222           1::::l               3:::::3   1::::l     H::::::HHHHH::::::H
 *! 2:::::2                1::::l               3:::::3   1::::l     H:::::H     H:::::H
 *! 2:::::2                1::::l               3:::::3   1::::l     H:::::H     H:::::H
 *! 2:::::2       222222111::::::1113333333     3:::::3111::::::111HH::::::H     H::::::HH
 *! 2::::::2222222:::::21::::::::::13::::::33333::::::31::::::::::1H:::::::H     H:::::::H
 *! 2::::::::::::::::::21::::::::::13:::::::::::::::33 1::::::::::1H:::::::H     H:::::::H
 *! 22222222222222222222111111111111 333333333333333   111111111111HHHHHHHHH     HHHHHHHHH
 * ------------------------------------------------------------------------------------------
 */

lemlib::PID armPID(3., 0.0, 2, 0, false);

Arm arm(
    &ArmConfig::motors,
    &ArmConfig::rotation,
    1.0,
    {0, 29, 155, 200, 225},
    pros::E_CONTROLLER_DIGITAL_R1,
    pros::E_CONTROLLER_DIGITAL_R2,
    &primary,
    &armPID);

Intake intake(
    &IntakeConfig::first,
    &IntakeConfig::second,
    &IntakeConfig::optical,
    &IntakeConfig::lift,
    307,
    pros::E_CONTROLLER_DIGITAL_L1,
    pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_RIGHT,
    &primary,
    35,
    160);

Clamp clamp(
    &ClampConfig::pneumatic,
    &ClampConfig::distance,
    100,
    2.0,
    pros::E_CONTROLLER_DIGITAL_X,
    &primary,
    false);

Doinkler doinklerLeft('D', false, pros::E_CONTROLLER_DIGITAL_DOWN, &primary);
Doinkler doinklerRight('E', false, pros::E_CONTROLLER_DIGITAL_B, &primary);

Terminal Console(15);

/**
 * -------------------------------------------------------------
 * ? @@@       @@@@@@@@  @@@@@@@@@@   @@@       @@@  @@@@@@@
 * ? @@@       @@@@@@@@  @@@@@@@@@@@  @@@       @@@  @@@@@@@@
 * ? @@!       @@!       @@! @@! @@!  @@!       @@!  @@!  @@@
 * ? !@!       !@!       !@! !@! !@!  !@!       !@!  !@   @!@
 * ? @!!       @!!!:!    @!! !!@ @!@  @!!       !!@  @!@!@!@
 * ? !!!       !!!!!:    !@!   ! !@!  !!!       !!!  !!!@!!!!
 * ? !!:       !!:       !!:     !!:  !!:       !!:  !!:  !!!
 * ? :!:       :!:       :!:     :!:  :!:       :!:  :!:  !:!
 * ? :: ::::   :: ::::   :::     ::   :: ::::    ::  :::  ::
 * ? : :: : :  : :: ::    :      :    : :: : :   :   :::::
 * -------------------------------------------------------------
 */

// Drivetrain Configuration
lemlib::Drivetrain drivetrain(&leftDrive, &rightDrive, 13.25, 2.75, 450, 8);

// tracking wheel Configuration
lemlib::TrackingWheel verticalWheel(&leftDrive, 2.75, -13.25 / 2.0, 450);

// Pass sensors to lemlib
lemlib::OdomSensors sensors(&verticalWheel, nullptr, nullptr, nullptr, &inertial);

// PID Configuration
lemlib::ControllerSettings lateralPID(
    11,
    0,
    8,
    3,
    1,    // IN INCHES
    100,  // IN MSEC
    2,    // In INCHES
    800,  // IN MSEC
    40);
lemlib::ControllerSettings angularPID(
    2,
    0,
    10,
    3,
    1,    // IN DEGREES
    100,  // IN MSEC
    3,    // IN DEGREES
    500,  // IN MSEC
    0);

// Chassis Definition
Chassis chassis({drivetrain, lateralPID, angularPID, sensors});