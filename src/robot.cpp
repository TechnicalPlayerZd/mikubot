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
        leftTop.move(leftPower);
        leftMiddle.move(leftPower);
        leftBack.move(leftPower);
        rightTop.move(rightPower);
        rightMiddle.move(rightPower);
        rightBack.move(rightPower);
    }

    void stop() {
        drive(0, 0); // Stop all motors by setting power to 0
    }

    void drive(int speed, int duration) {
        // Reduce speed for slower movement
        int reducedSpeed = speed / 2;

        // Set motor directions
        leftTop.set_reversed(false);
        leftMiddle.set_reversed(true);
        leftBack.set_reversed(true);
        rightTop.set_reversed(true);
        rightMiddle.set_reversed(false);
        rightBack.set_reversed(false);

        // Move motors at reduced speed
        leftTop.move(reducedSpeed);
        leftMiddle.move(reducedSpeed);
        leftBack.move(reducedSpeed);
        rightTop.move(reducedSpeed);
        rightMiddle.move(reducedSpeed);
        rightBack.move(reducedSpeed);

        // Delay for the specified duration
        pros::delay(duration);

        // Apply braking
        leftTop.move(-10);
        leftMiddle.move(-10);
        leftBack.move(-10);
        rightTop.move(-10);
        rightMiddle.move(-10);
        rightBack.move(-10);
        pros::delay(100); // Short brake duration

        // Stop motors
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
    void turnDegrees(double degrees, int speed = 60) {
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
    
    // Solenoid controls
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      solenoidStateC = !solenoidStateC;
      solenoidC.set_value(solenoidStateC);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // Top Outake
      intakeTop.move(127);
      intakeMiddle.move(-127);
      intakeBottom.move(-127);
      outtakeRear.move(-127);
      solenoidA.set_value(false);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // Middle Outake
      intakeTop.move(-127);
      intakeMiddle.move(-127);
      intakeBottom.move(-127);
      outtakeRear.move(-127);
      solenoidA.set_value(false);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // Lower Outake
      intakeTop.move(-127);
      intakeMiddle.move(-127);
      intakeBottom.move(127);
      outtakeRear.move(-127);
      solenoidA.set_value(false);
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

void robotInitialize() {
  pros::lcd::initialize(); // Initialize the LCD screen
  odom::start();
  pros::Task displayTask(displayPosition); // Start position display task
}