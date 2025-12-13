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
    // Remove the drive function
    // Remove the turnDegrees function

    void stop() {
        // Stop all motors by setting power to 0
        leftTop.move(0);
        leftMiddle.move(0);
        leftBack.move(0);
        rightTop.move(0);
        rightMiddle.move(0);
        rightBack.move(0);
    }

    void intake(int speed) {
        intakeTop.move(speed);
        intakeMiddle.move(-speed);
        intakeBottom.move(-speed);
    }

    // ...existing code...
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

    static bool solenoidStateA = true;
    static bool solenoidStateB = true;
    static bool solenoidStateC = false;

    while (true) {
        solenoidB.set_value(solenoidStateB);
        solenoidA.set_value(solenoidStateA);
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
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            solenoidStateB = !solenoidStateB;
            solenoidB.set_value(solenoidStateB);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Middle Outtake
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
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Lower Outtake
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

    // Toggle solenoid B to ensure it is reset
    solenoidB.set_value(false);
    pros::delay(100); // Wait for 100ms to ensure the solenoid reacts
    solenoidB.set_value(true); // Set solenoid B to false (turn down)

    // Debug print to confirm the value is set
    pros::lcd::print(0, "Solenoid B toggled and set to false");
}