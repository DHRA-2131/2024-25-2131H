#pragma once
#include "lemlib/api.hpp"
#include "pros/adi.hpp"

// Constants
constexpr double driveRPM = 450;
constexpr double trackWidth = 13.5;
constexpr double armHeight = 300;

// Team Color Toggle
extern pros::adi::DigitalIn teamColor;

// Drivetrain
extern pros::v5::MotorGroup leftDrive;
extern pros::v5::MotorGroup rightDrive;
extern pros::Imu imu;
extern pros::adi::Pneumatics clamp;

// Arm
extern pros::v5::Motor arm;
extern pros::v5::Rotation armEncoder;
extern pros::adi::Pneumatics doinkler;

// Intake
extern pros::v5::Motor intake;

// Controller
extern pros::Controller primary;

// LEMLIB
// Drivetrain
extern lemlib::Drivetrain drivetrain;

// Odometry Config
extern lemlib::TrackingWheel leftTrackingWheel;
extern lemlib::TrackingWheel rightTrackingWheel;
extern lemlib::OdomSensors sensors;

// PID Controllers
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

// Chassis Class
extern lemlib::Chassis chassis;