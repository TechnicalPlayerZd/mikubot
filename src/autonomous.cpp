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
        intake(127); // Start intake at full speed
        pros::delay(1000); // Run intake for the same duration as driving
        intake(0); // Stop intake
    });

    // Drive forward 50 cm without PID
    drive(127, 1000); // Speed: 127, Duration: 1000 ms
    stop();

    // Turn 135 degrees (180 - 45) to the right
    turnDegrees(135);

    // Move forward 908.29 cm
    drive(908.29, 1000);

    // Turn 135 degrees (180 - 45) to the left
    turnDegrees(-135);

    // Move forward 574.427 cm
    drive(574.427, 1000);

    // Activate outtake
    intake(-127); // Reverse intake for outtake
    pros::delay(1000); // Run outtake for 1 second
    intake(0); // Stop outtake

    // Ensure intake task finishes
    intake_task.remove();
}

} // namespace robot
