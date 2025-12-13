#include "diy/autonomous.hpp"
#include "robot.hpp"
#include "diy/pid.hpp"
#include "odom.hpp"
#include "pros/misc.h"

namespace robot {

// Simplified drive function for debugging
void drive(double distanceCm, int maxSpeed) {
    const double wheelDiameterCm = 5.6; // Adjust based on your robot's wheel diameter
    const double wheelCircumferenceCm = wheelDiameterCm * M_PI;
    const double degreesPerCm = 360.0 / wheelCircumferenceCm;

    double targetDegrees = distanceCm * degreesPerCm;

    leftTop.tare_position();
    rightTop.tare_position();

    // Debug print
    pros::lcd::print(0, "Driving %.2f cm at %d speed", distanceCm, maxSpeed);

    const double kP = 0.5; // Proportional constant (tune this value)
    while (true) {
        double leftPosition = leftTop.get_position();
        double rightPosition = rightTop.get_position();
        double averagePosition = (leftPosition + rightPosition) / 2.0;

        double error = targetDegrees - averagePosition;
        if (std::fabs(error) < 5) { // Stop if the error is small enough
            break;
        }

        int speed = std::clamp(static_cast<int>(error * kP), -maxSpeed, maxSpeed);

        leftTop.move_velocity(speed);
        leftMiddle.move_velocity(speed);
        leftBack.move_velocity(speed);
        rightTop.move_velocity(speed);
        rightMiddle.move_velocity(speed);
        rightBack.move_velocity(speed);

        pros::delay(10); // Small delay for control loop
    }

    stop(); // Stop the robot after reaching the target
    pros::delay(500); // Add a delay to ensure the robot stops completely
}

// Simplified turnDegrees function for debugging
void turnDegrees(double degrees, int maxSpeed) {
    const double turnFactor = 5.6; // Adjust this factor based on your robot's turning behavior
    double targetTurnDegrees = degrees * turnFactor;

    leftTop.tare_position();
    rightTop.tare_position();

    // Debug print
    pros::lcd::print(0, "Turning %.2f degrees at %d speed", degrees, maxSpeed);

    const double kP = 0.5; // Proportional constant (tune this value)
    while (true) {
        double leftPosition = leftTop.get_position();
        double rightPosition = rightTop.get_position();

        double leftError = -targetTurnDegrees - leftPosition;
        double rightError = targetTurnDegrees - rightPosition;

        if (std::fabs(leftError) < 5 && std::fabs(rightError) < 5) { // Stop if the error is small enough
            break;
        }

        int leftSpeed = std::clamp(static_cast<int>(leftError * kP), -maxSpeed, maxSpeed);
        int rightSpeed = std::clamp(static_cast<int>(rightError * kP), -maxSpeed, maxSpeed);

        leftTop.move_velocity(leftSpeed);
        leftMiddle.move_velocity(leftSpeed);
        leftBack.move_velocity(leftSpeed);
        rightTop.move_velocity(rightSpeed);
        rightMiddle.move_velocity(rightSpeed);
        rightBack.move_velocity(rightSpeed);

        pros::delay(10); // Small delay for control loop
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
    robot::drive(25.0, 100);
    pros::lcd::print(1, "Completed drive 25.0 cm");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(2, "Starting turn 135 degrees");
    robot::turnDegrees(135, 50);
    pros::lcd::print(2, "Completed turn 135 degrees");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(3, "Starting drive 908.29 cm");
    robot::drive(908.29, 100);
    pros::lcd::print(3, "Completed drive 908.29 cm");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(4, "Starting turn -135 degrees");
    robot::turnDegrees(-135, 50);
    pros::lcd::print(4, "Completed turn -135 degrees");
    pros::delay(500); // Ensure the robot stops completely

    pros::lcd::print(5, "Starting drive 574.427 cm");
    robot::drive(574.427, 100);
    pros::lcd::print(5, "Completed drive 574.427 cm");
    pros::delay(500); // Ensure the robot stops completely

    // Debug print
    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE, 6, "Autonomous movements completed");
}

} // namespace robot
