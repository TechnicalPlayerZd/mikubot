#include "robot.hpp"
#include "odom.hpp"
#include "pros/misc.hpp" // Include the pros/misc.hpp header
#include "diy/autonomous.hpp"

// === Motor Setup ===
pros::Motor leftTop(7, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor leftMiddle(8, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor leftBack(13, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

pros::Motor rightTop(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor rightMiddle(2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor rightBack(18, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// Intake
pros::Motor intakeTop(12, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intakeBottom(14, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intakeMiddle(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor outtakeRear(10, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// Sensors
pros::Rotation odomH(5);
pros::Rotation odomV(6);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut solenoidA('A');
pros::adi::DigitalOut solenoidB('B');
pros::adi::DigitalOut solenoidC('C');

// Define rightReversed as a constant

extern pros::Mutex odomMutex; // Declare the mutex

namespace robot {
    void drive(double leftPower, double rightPower) {
        // Directly set motor power (-127 to 127)
        leftTop.move(leftPower);
        leftMiddle.move(leftPower);
        leftBack.move(leftPower);
        rightTop.move(rightPower);
        rightMiddle.move(rightPower);
        rightBack.move(rightPower);
    }

    void stop() {
        // Stop all motors by setting power to 0
        leftTop.move(0);
        leftMiddle.move(0);
        leftBack.move(0);
        rightTop.move(0);
        rightMiddle.move(0);
        rightBack.move(0);
    }

    void drive(double distanceCm, int speed) {
        const double wheelDiameterCm = 5.6; // Adjust based on your robot's wheel diameter
        const double wheelCircumferenceCm = wheelDiameterCm * M_PI;
        const double degreesPerCm = 360.0 / wheelCircumferenceCm;
        

        double targetDegrees = distanceCm * degreesPerCm;

        leftTop.tare_position();
        rightTop.tare_position();

        leftTop.move_relative(targetDegrees, speed);
        leftMiddle.move_relative(targetDegrees, speed);
        leftBack.move_relative(targetDegrees, speed);
        rightTop.move_relative(targetDegrees, speed);
        rightMiddle.move_relative(targetDegrees, speed);
        rightBack.move_relative(targetDegrees, speed);
        leftTop.set_reversed(false);
        leftMiddle.set_reversed(true);
        leftBack.set_reversed(true);

        rightTop.set_reversed(true);
        rightMiddle.set_reversed(false);
        rightBack.set_reversed(false);

        while (std::fabs(leftTop.get_position()) < std::fabs(targetDegrees) ||
               std::fabs(rightTop.get_position()) < std::fabs(targetDegrees)) {
            pros::delay(10); // Wait until the target position is reached
        }

        stop(); // Stop the robot after reaching the target
    }

    void intake(int speed) {
        intakeTop.move(speed);
        intakeMiddle.move(-speed);
        intakeBottom.move(-speed);
    }

    void turnDegrees(double degrees, int speed) {
        double targetAngle = odom::getTheta() + degrees;

        // Normalize target angle to [-180, 180]
        while (targetAngle > 180.0) targetAngle -= 360.0;
        while (targetAngle < -180.0) targetAngle += 360.0;

        while (true) {
            double currentAngle = odom::getTheta();
            double error = targetAngle - currentAngle;

            // Normalize error to [-180, 180]
            while (error > 180.0) error -= 360.0;
            while (error < -180.0) error += 360.0;

            if (std::fabs(error) < 1.0) break; // Stop when within 1 degree tolerance

            int turnPower = (error > 0 ? speed : -speed);
            drive(-turnPower, turnPower); // Turn in place

            pros::delay(20);
        }

        stop(); // Stop the robot after turning
    }

    void drive(int speed, int duration) {
        // Set motor directions
        leftTop.set_reversed(false);
        leftMiddle.set_reversed(true);
        leftBack.set_reversed(true);
        rightTop.set_reversed(true);
        rightMiddle.set_reversed(false);
        rightBack.set_reversed(false);

        // Move motors at the specified speed
        leftTop.move(speed);
        leftMiddle.move(speed);
        leftBack.move(speed);
        rightTop.move(speed);
        rightMiddle.move(speed);
        rightBack.move(speed);

        // Delay for the specified duration
        pros::delay(duration);

        // Stop motors
        stop();
    }
}

// === Operator Control (Drivetrain Test) ===
void robotOpcontrol() {
    // Reverse the left side manually for correct drive direction
    leftTop.set_reversed(false);
    leftMiddle.set_reversed(true);
    leftBack.set_reversed(true);

    rightTop.set_reversed(true);
    rightMiddle.set_reversed(false);
    rightBack.set_reversed(false);

    static bool solenoidStateA = false;
    static bool solenoidStateB = true;
    static bool solenoidStateC = false;

    while (true) {
        int leftPower = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Apply motor power
        leftTop.move(leftPower);
        leftMiddle.move(leftPower);
        leftBack.move(leftPower);
        rightTop.move(rightPower);
        rightMiddle.move(rightPower);
        rightBack.move(rightPower);

        // Monitor motor tension using current draw and velocity
        double outtakeRearCurrent = outtakeRear.get_current_draw();
        double intakeBottomCurrent = intakeBottom.get_current_draw();
        double outtakeRearVelocity = outtakeRear.get_actual_velocity();
        double intakeBottomVelocity = intakeBottom.get_actual_velocity();

        // Increase the current threshold to 3000 mA
        bool isUnderTension = (outtakeRearCurrent > 3000 || intakeBottomCurrent > 3000) && // High current
                              (std::fabs(outtakeRearVelocity) < 5.0 || std::fabs(intakeBottomVelocity) < 5.0); // Low velocity

        // Solenoid controls
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            solenoidStateC = !solenoidStateC;
            solenoidC.set_value(solenoidStateC);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Middle Outtake
            if (isUnderTension) {
                outtakeRear.move(127); // Reverse outtakeRear
                intakeBottom.move(127); // Reverse intakeBottom
            } else {
                intakeTop.move(127);
                intakeMiddle.move(-127);
                intakeBottom.move(-127);
                outtakeRear.move(-127);
                solenoidA.set_value(true);
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // Top Outtake
            if (isUnderTension) {
                outtakeRear.move(127); // Reverse outtakeRear
                intakeBottom.move(127); // Reverse intakeBottom
            } else {
                intakeTop.move(-127);
                intakeMiddle.move(-127);
                intakeBottom.move(-127);
                outtakeRear.move(-127);
                solenoidA.set_value(false);
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Lower Outtake
            if (isUnderTension) {
                outtakeRear.move(127); // Reverse outtakeRear
                intakeBottom.move(127); // Reverse intakeBottom
            } else {
                intakeTop.move(-127);
                intakeMiddle.move(-127);
                intakeBottom.move(127);
                outtakeRear.move(-127);
                solenoidA.set_value(true);
            }
        } else { // Automatic Intake
            intakeTop.move(-127); // Adjust the power as needed
            intakeMiddle.move(-127);
            intakeBottom.move(-127);
            outtakeRear.move(127);
            solenoidA.set_value(true);
        }

        // Add a small delay to prevent blocking other tasks
        pros::delay(20);
    }
}

void stop() {
    leftTop.move(0);
    leftMiddle.move(0);
    leftBack.move(0);
    rightTop.move(0);
    rightMiddle.move(0);
    rightBack.move(0);
}

void displayPosition() {
  pros::lcd::initialize(); // Ensure the LCD is initialized
  while (true) {
    odomMutex.take(); // Lock the mutex
    double x = odom::getX();
    double y = odom::getY();
    double theta = odom::getTheta();
    odomMutex.give(); // Unlock the mutex

    pros::lcd::clear_line(0);
    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    pros::lcd::print(0, "X: %.2f cm", x);
    pros::lcd::print(1, "Y: %.2f cm", y);
    pros::lcd::print(2, "Theta: %.2f deg", theta);

    pros::delay(100); // Update every 100ms
  }
}

void displayOdometryOnBrain() {
    pros::lcd::initialize(); // Ensure the LCD is initialized
    while (true) {
        odomMutex.take(); // Lock the mutex
        double x_mm = odom::getXmm(); // Get X in millimeters
        double y_mm = odom::getYmm(); // Get Y in millimeters
        double theta = odom::getTheta(); // Get theta in degrees
        odomMutex.give(); // Unlock the mutex

        pros::lcd::clear_line(0);
        pros::lcd::clear_line(1);
        pros::lcd::clear_line(2);
        pros::lcd::print(0, "X: %.1f mm", x_mm);
        pros::lcd::print(1, "Y: %.1f mm", y_mm);
        pros::lcd::print(2, "Theta: %.1f deg", theta);
        pros::delay(100); // Update every 100ms
    }
}

void robotInitialize() {
  pros::lcd::initialize(); // Initialize the LCD screen
  odom::start();
  pros::Task displayTask(displayOdometryOnBrain); // Start the display task
}