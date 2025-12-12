#include "diy/autonomous.hpp"
#include "robot.hpp"
#include "diy/pid.hpp"
#include "odom.hpp"
#include "pros/misc.h"

namespace robot {

void autonomous() {
    // Start odometry background task
    odom::start();

    // Reset robot pose to origin (0,0), facing +Y
    odom::reset();
    pros::delay(100); // Allow sensors to stabilize

    // Activate intake while driving forward
    pros::Task intake_task([]() {
        robot::intake(127); // Start intake at full speed
        pros::delay(1000); // Run intake for the same duration as driving
        robot::intake(0); // Stop intake
    });

    // Drive forward 74 cm using the distance-based drive function
    robot::drive(74.0, 100);

    // Turn 135 degrees (180 - 45) to the right
    robot::turnDegrees(135);

    // Move forward 908.29 cm
    robot::drive(908.29, 100);

    // Turn 135 degrees (180 - 45) to the left
    robot::turnDegrees(-135);

    // Move forward 574.427 cm
    robot::drive(574.427, 100);

    // Activate outtake
    robot::intake(-127); // Reverse intake for outtake
    pros::delay(1000); // Run outtake for 1 second
    robot::intake(0); // Stop outtake

    // Ensure intake task finishes
    intake_task.remove();
}

} // namespace robot
