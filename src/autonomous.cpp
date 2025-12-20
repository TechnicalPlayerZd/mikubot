#include "diy/autonomous.hpp"
#include "robot.hpp"
#include "diy/pid.hpp"
#include "odom.hpp"
#include "pros/misc.h"

namespace robot {

// Simplified drive function for debugging
void drive(double distanceCm, int speed) {
    const double time_tuning = 5.6; // Adjust to testing
    const double wheelCircumferenceCm = wheelDiameterCm * M_PI;
    const double degreesPerCm = 360.0 / wheelCircumferenceCm;

    double targetDegrees = distanceCm * degreesPerCm;

    leftTop.tare_position();
    rightTop.tare_position();

    // Debug print
    pros::lcd::print(0, "Driving %.2f cm at %d speed", distanceCm, speed);
    pros::lcd::print(1, "Target Degrees: %.2f", targetDegrees);

    // Set motor brake mode to coast
    leftTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMiddle.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMiddle.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Use move_velocity for debugging
    leftTop.move_velocity(speed);
    leftMiddle.move_velocity(speed);
    leftBack.move_velocity(speed);
    rightTop.move_velocity(speed);
    rightMiddle.move_velocity(speed);
    rightBack.move_velocity(speed);

    pros::delay(distanceCm*time_tuning); // Run motors for the distance times a constant(testing)

    stop(); // Stop the robot after reaching the target
    pros::delay(500); // Add a delay to ensure the robot stops completely
}

// Simplified turnDegrees function for debugging
void turnDegrees(double degrees, int speed) {
    const double turnFactor = 5.6 / 2.67; // Adjusted turn factor for accurate turning
    double targetTurnDegrees = degrees * turnFactor;

    leftTop.tare_position();
    rightTop.tare_position();

    // Debug print
    pros::lcd::print(0, "Turning %.2f degrees at %d speed", degrees, speed);

    leftTop.move_relative(-targetTurnDegrees, speed);
    leftMiddle.move_relative(-targetTurnDegrees, speed);
    leftBack.move_relative(-targetTurnDegrees, speed);
    rightTop.move_relative(targetTurnDegrees, speed);
    rightMiddle.move_relative(targetTurnDegrees, speed);
    rightBack.move_relative(targetTurnDegrees, speed);

    uint32_t startTime = pros::millis();
    while ((std::fabs(leftTop.get_position()) < std::fabs(targetTurnDegrees) ||
            std::fabs(rightTop.get_position()) < std::fabs(targetTurnDegrees)) &&
           (pros::millis() - startTime < 5000)) { // Timeout after 5 seconds
        // Debug current motor positions
        pros::lcd::print(1, "Left Pos: %.2f", leftTop.get_position());
        pros::lcd::print(2, "Right Pos: %.2f", rightTop.get_position());
        pros::delay(10); // Wait until the target position is reached
    }

    stop(); // Stop the robot after turning
    pros::delay(500); // Add a delay to ensure the robot stops completely
}

void autonomous() {
    // Start odometry background task
    odom::start();

    // Reset robot pose to origin (0,0), facing +Y
    odom::reset();
    pros::delay(100); // Allow sensors to stabilize

    // Debug print
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE, 0, "Autonomous started");

    // Configure motor reversals for autonomous
    leftTop.set_reversed(false);
    leftMiddle.set_reversed(true);
    leftBack.set_reversed(true);

    rightTop.set_reversed(true);
    rightMiddle.set_reversed(false);
    rightBack.set_reversed(false);

    // Follow the specified movements
    pros::lcd::print(1, "Starting drive 25.0 cm");
    robot::drive(25.0 / 3.0, 100); // Ensure the first argument is a double
    pros::lcd::print(1, "Completed drive 25.0 cm");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(2, "Starting turn 135 degrees");
    robot::turnDegrees(135, 50);
    pros::lcd::print(2, "Completed turn 135 degrees");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(3, "Starting drive 908.29 cm");
    robot::drive(908.29 / 3.0, 100); // Ensure the first argument is a double
    pros::lcd::print(3, "Completed drive 908.29 cm");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(4, "Starting turn -135 degrees");
    robot::turnDegrees(-135, 50);
    pros::lcd::print(4, "Completed turn -135 degrees");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(5, "Starting drive 574.427 cm");
    robot::drive(574.427 / 3.0, 100); // Ensure the first argument is a double
    pros::lcd::print(5, "Completed drive 574.427 cm");
    pros::delay(500); // Ensure the robot stops completely

    // Debug print
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE, 6, "Autonomous movements completed");
}

} // namespace robot
