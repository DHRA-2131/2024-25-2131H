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
#include "2131H/Utilities/ArmPID.hpp"
#include "2131H/Utilities/Console.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
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

// LEFT {PORTS (NOT ORDERED) "-" To reverse}, Cart, Encoder Unit
pros::MotorGroup leftDrive(
    {-16, -17, -18}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// RIGHT {PORTS (NOT ORDERED) "-" To reverse}, Cart, Encoder Unit
pros::MotorGroup rightDrive(
    {8, 9, 10}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

// Inertial Port
pros::Imu inertial(20);

namespace IntakeConfig
{
// First Stage (5.5W) Port, Cart, Encoder Unit
pros::Motor first(-15, pros::MotorGears::green, pros::v5::MotorEncoderUnits::deg);

// Second Stage (11W) Port, Cart, Encoder Unit
pros::Motor second(7, pros::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);

// Color Sort Optical
pros::Optical optical(14);
// Intake Lift Port
pros::adi::Pneumatics lift('D', false, false);
// Eject
pros::adi::DigitalIn eject('B');
}  // namespace IntakeConfig

namespace ArmConfig
{
// Arm {PORTS (NOT ORDERED) "-" To reverse}, Cart, Encoder Unit
pros::MotorGroup motors({11}, pros::MotorGears::green, pros::v5::MotorEncoderUnits::deg);
// Rotational Port
pros::Rotation rotation(-12);
}  // namespace ArmConfig

namespace ClampConfig
{
// Clamp Pneumatic Port
pros::adi::Pneumatics pneumatic('A', false, false);
// AutoClamp (Tm) distance sensor
pros::Distance distance(19);
}  // namespace ClampConfig

// Controller (Primary)
pros::Controller primary(pros::E_CONTROLLER_MASTER);

// Screen Selector Buttons
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

// Proportional, Integral, Derivative, Gravity, Windup, Sign Flip //* Tune kG to add anti-gravity
// term
Utilities::ArmPID armPID(3., 0.0, 2, 0, 0, false);

Arm arm(
    &ArmConfig::motors,      // Pointer to motors
    &ArmConfig::rotation,    // Pointer to rotational
    1.0,                     // Ratio from motor to arm //* Not Used if rotational
    {0, 29, 155, 200, 225},  // Macro'd Arm Positions //*This will need tuning
    0,  // Reading when arm is completely vertical //* If kG != 0 then this needs tuning
    pros::E_CONTROLLER_DIGITAL_R1,  // Button for up
    pros::E_CONTROLLER_DIGITAL_R2,  // Button for down
    &primary,                       // Controller Pointer
    &armPID                         // PID Pointer
);

Intake intake(
    &IntakeConfig::first,    // Pointer to first motor
    &IntakeConfig::second,   // Pointer to second motor
    &IntakeConfig::optical,  // Pointer to color sort optical
    &IntakeConfig::lift,     // Pointer to intake lift
    &IntakeConfig::eject,
    pros::E_CONTROLLER_DIGITAL_L1,     // Button for intake
    pros::E_CONTROLLER_DIGITAL_L2,     // Button for outtake
    pros::E_CONTROLLER_DIGITAL_RIGHT,  // Button for lift
    &primary,                          // Controller Pointer
    35,                                // Anything Less than this value is a red ring
    160                                // Anything Greater than this value is a blue ring
);

Clamp clamp(
    &ClampConfig::pneumatic,       // Pneumatic Pointer
    &ClampConfig::distance,        // Auto Clamp Pointer
    100,                           // Trigger Distance //* Tune this for auto clamp
    2.0,                           //* Can be ignored
    pros::E_CONTROLLER_DIGITAL_X,  // Clamp button
    &primary,                      // Controller Pointer
    false                          // Auto Clamp Enabled by default?
);

// Left Doinkler Port, Start Extended, Button, Controller Pointer
Doinkler doinklerLeft('F', false, pros::E_CONTROLLER_DIGITAL_DOWN, &primary);
// Right Doinkler Port, Start Extended, Button, Controller Pointer
Doinkler doinklerRight('C', false, pros::E_CONTROLLER_DIGITAL_B, &primary);

Doinkler hang('E', false, pros::E_CONTROLLER_DIGITAL_RIGHT, &primary);

//* Can ignore
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
lemlib::Drivetrain drivetrain(
    &leftDrive,   // Left MotorGroup Pointer
    &rightDrive,  // Right MotorGroup Pointer
    13.25,        // TrackWidth (Distance to center of wheels) //* Check this
    2.75,         // Wheel diameter //* Measure wheels and avg
    450,          // RPM of drive
    8             // * Can ignore for now
);

// tracking wheel Configuration
lemlib::TrackingWheel verticalWheel(
    &leftDrive,    // Left MotorGroup Pointer
    2.75,          // Wheel Diameter //* use value you found above
    -13.25 / 2.0,  // Distance from wheel to tracking center
    450            // RPM of tracking wheel
);

// Pass sensors to lemlib
lemlib::OdomSensors sensors(
    &verticalWheel,  // Vertical Tracking Wheel Pointer 1
    nullptr,         // Vertical Tracking Wheel Pointer 2
    nullptr,         // Horizontal Tracking Wheel Pointer 1
    nullptr,         // Horizontal Tracking Wheel Pointer 2
    &inertial        // Inertial Pointer
);

// PID Configuration //* These need to be tuned before autos
lemlib::ControllerSettings lateralPID(
    11,   // Lateral kP
    0,    // Lateral kI
    8,    // Lateral kD
    3,    // Windup Range
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
Chassis chassis({
    drivetrain,  // Drivetrain Struct
    lateralPID,  // lateral PID struct
    angularPID,  // angular PID struct
    sensors      // Odometry sensors struct
});