#include "main/RobotConfig.hpp"

namespace Screen
{
pros::adi::DigitalIn teamColor('A');
}  // namespace Screen
namespace Systems
{

namespace Drivetrain
{
// Left Drive
pros::v5::MotorGroup leftDrive({-18, -19, -20}, pros::v5::MotorGearset::blue);
// Right Drive
pros::v5::MotorGroup rightDrive({6, 9, 10}, pros::v5::MotorGearset::blue);
// Vex V5 Inertial Sensor
pros::Imu imu(22);
}  // namespace Drivetrain

namespace Clamp
{
// Clamp Pneumatic
pros::adi::Pneumatics pneumatic('G', false, false);
// Distance sensor to detect goals
pros::Distance goalDetector(22);
}  // namespace Clamp

namespace Arm
{
// Arm Motor
pros::v5::Motor motor(12, pros::MotorGear::red);

// Doinkler (For removing corner rings)
pros::adi::Pneumatics doinkler('H', false, false);
}  // namespace Arm

namespace Intake
{
// Intake Motor
pros::v5::Motor motor(11, pros::v5::MotorGearset::blue);
// Vex V5 Optical Sensor (For detecting ring colors)
pros::Optical colorDetector(22);
// Vex V5 Distance Sensor (For detecting rings as they approach the top of the intake)
//* Two sensors are being used due to the refresh rate on Vex V5 Optical Sensors.
//* Optical sensors update at 100 Msec and Distance sensors update at ~50 msec
pros::Distance ringDetector(22);

// Pneumatic discard / eject
pros::adi::Pneumatics ringSort('A', false, false);
}  // namespace Intake

}  // namespace Systems

// Controllers (Only run one)
pros::Controller primary(pros::E_CONTROLLER_MASTER);

// *** === LEMLIB UTILIZATION === *** //

// Drivetrain
lemlib::Drivetrain drivetrain(&Systems::Drivetrain::leftDrive,   // left motor group
                              &Systems::Drivetrain::rightDrive,  // right motor group
                              13.25,                             // 13.25 inch track width
                              3.25,                              // using new 3.25" omniwheel
                              driveRPM,                          // drivetrain rpm is 450
                              8                                  // Traction Wheel drive
);

// Odometry
// horizontal tracking wheel
lemlib::TrackingWheel leftTrackingWheel(&Systems::Drivetrain::leftDrive, 3.25, -6.625, 450);
// vertical tracking wheel
lemlib::TrackingWheel rightTrackingWheel(&Systems::Drivetrain::rightDrive, 3.25, 6.625, 450);

lemlib::OdomSensors sensors(&leftTrackingWheel, &rightTrackingWheel, nullptr, nullptr, &Systems::Drivetrain::imu);

// PID Controllers
// lateral PID controller
lemlib::ControllerSettings lateral_controller(6.41601563,  // proportional gain (kP)
                                              0,           // integral gain (kI)
                                              0.24023438,  // derivative gain (kD)
                                              3,           // anti windup
                                              1,           // small error range, in inches
                                              10000,       // small error range timeout, in milliseconds
                                              3,           // large error range, in inches
                                              50000,       // large error range timeout, in milliseconds
                                              20           // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.005,  // proportional gain (kP)
                                              0,      // integral gain (kI)
                                              0.5,    // derivative gain (kD)
                                              3,      // anti windup
                                              1,      // small error range, in degrees
                                              100,    // small error range timeout, in milliseconds
                                              3,      // large error range, in degrees
                                              500,    // large error range timeout, in milliseconds
                                              0       // maximum acceleration (slew)
);

// Chassis class
Pathing::Chassis chassis(drivetrain,          // drivetrain settings
                         lateral_controller,  // lateral PID settings
                         angular_controller,  // angular PID settings
                         sensors              // odometry sensors
);
