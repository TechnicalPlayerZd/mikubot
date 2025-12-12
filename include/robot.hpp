#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include "main.h"
#include "diy/autonomous.hpp" // Include autonomous header

// Declare motors
extern pros::Motor leftTop;
extern pros::Motor leftMiddle;
extern pros::Motor leftBack;
extern pros::Motor rightTop;
extern pros::Motor rightMiddle;
extern pros::Motor rightBack;

// Declare intake motors
extern pros::Motor intakeTop;
extern pros::Motor intakeBottom;

// Declare sensors
extern pros::Rotation odomH;
extern pros::Rotation odomV;

// Declare controller
extern pros::Controller master;

// Declare pneumatics
extern pros::adi::DigitalOut solenoid;

// Rename opcontrol function to robotOpcontrol
void robotOpcontrol();

// Declare robotInitialize function
void robotInitialize();

namespace robot {
    void drive(double leftPower, double rightPower); // Declare drive function
    void drive(int speed, int duration); // Drive function without PID
    void drive(double distanceCm, int speed); // Drive function for distance in cm
    void intake(int speed); // Declare intake function
    void stop(); // Declare stop function
    void turnDegrees(double degrees, int speed = 60); // Declare turnDegrees function
}

#endif  // _ROBOT_HPP_
