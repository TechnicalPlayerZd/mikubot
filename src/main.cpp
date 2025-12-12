#include "main.h"
#include "robot.hpp"
#include "pros/apix.h"

// Initialize function
void initialize() {
  robotInitialize();
  // Add any initialization code here if needed
}

// Disabled function
void disabled() {
  // Add any code to run when the robot is disabled
}

// Competition initialization function
void competition_initialize() {
  // Add any pre-autonomous setup code here
}

// Define the autonomous function
void autonomous() {
    robot::autonomous(); // Correctly call the autonomous function from the robot namespace
}

// Operator control function
void opcontrol() {
  // Call the renamed robotOpcontrol function
  robot::autonomous();
  // robotOpcontrol();
}
