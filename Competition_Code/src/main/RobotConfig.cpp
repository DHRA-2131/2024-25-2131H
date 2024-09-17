#include "main/RobotConfig.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"

// Drivetrain
// -8, -9, -10
// 18, 19, 20
pros::v5::MotorGroup leftDrive({-8, -9, -7}, pros::v5::MotorGearset::blue);
pros::v5::MotorGroup rightDrive({18, 19, 20}, pros::v5::MotorGearset::blue);
pros::Imu imu(21);
pros::adi::Pneumatics clamp('C', false, false);

// Arm
pros::v5::Motor arm(-13, pros::MotorGear::red);
// Intake
pros::v5::Motor intake(3, pros::v5::MotorGearset::blue);

// Controller
pros::Controller primary(pros::E_CONTROLLER_MASTER);

// LEMLIB

// Drivetrain
lemlib::Drivetrain drivetrain(&leftDrive,                  // left motor group
                              &rightDrive,                 // right motor group
                              13.5,                        // 13.5 inch track width
                              lemlib::Omniwheel::NEW_275,  // using new 2.75" omniwheel
                              driveRPM,                    // drivetrain rpm is 450
                              2  // horizontal drift is 2 (for now)
);

// Odometry
// horizontal tracking wheel
lemlib::TrackingWheel leftTrackingWheel(&leftDrive, lemlib::Omniwheel::NEW_275, -5.75,
                                        450);
// vertical tracking wheel
lemlib::TrackingWheel rightTrackingWheel(&rightDrive, lemlib::Omniwheel::NEW_275, -2.5,
                                         450);

lemlib::OdomSensors sensors(&leftTrackingWheel, &rightTrackingWheel, nullptr, nullptr,
                            &imu);

// PID Controllers
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    6.41601563,  // proportional gain (kP)
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
lemlib::ControllerSettings angular_controller(
    1.005,  // proportional gain (kP)
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
lemlib::Chassis chassis(drivetrain,          // drivetrain settings
                        lateral_controller,  // lateral PID settings
                        angular_controller,  // angular PID settings
                        sensors              // odometry sensors
);
