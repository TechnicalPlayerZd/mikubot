#include "diy/pid.hpp"
#include "robot.hpp"
#include "odom.hpp"
#include <cmath>

// PID Gains
static constexpr double KP_DRIVE = 3.2;
static constexpr double KD_DRIVE = 0.8;
static constexpr double KP_TURN  = 6.0;
static constexpr double KD_TURN  = 1.2;

// Helper to clamp values
static inline double clamp(double val, double lo, double hi) {
    return (val < lo) ? lo : (val > hi) ? hi : val;
}

// Convert odometry heading to radians
static inline double getHeadingRad() {
    odomMutex.take(); // Lock the mutex
    double heading = odom::getTheta() * M_PI / 180.0;
    odomMutex.give(); // Unlock the mutex
    return heading;
}

// ---------------------------
// Drive a relative distance (cm)
// ---------------------------
void pid::driveTo(double targetDistance) {
    const double tolerance = 0.6; // cm
    double prevErr = 0.0;

    odomMutex.take(); // Lock the mutex
    double startX = odom::getX();
    double startY = odom::getY();
    odomMutex.give(); // Unlock the mutex

    uint32_t startMillis = pros::millis();

    while (true) {
        odomMutex.take(); // Lock the mutex
        double dx = odom::getX() - startX;
        double dy = odom::getY() - startY;
        odomMutex.give(); // Unlock the mutex

        double dist = std::hypot(dx, dy);

        double err = targetDistance - dist;
        double deriv = (err - prevErr) / 0.02;
        prevErr = err;

        double power = KP_DRIVE * err + KD_DRIVE * deriv;
        power = clamp(power, -127.0, 127.0);

        // Keep heading straight
        double headingErr = getHeadingRad(); // relative to start orientation
        double correction = KP_TURN * headingErr;
        correction = clamp(correction, -20, 20);

        robot::drive(power - correction, power + correction);

        if (std::fabs(err) < tolerance) break;
        if ((pros::millis() - startMillis) > 8000) break;

        pros::delay(20);
    }

    robot::stop();
}

// ---------------------------
// Turn to absolute heading (radians)
// ---------------------------
void pid::turnTo(double targetAngleRad) {
    const double toleranceDeg = 2.0; // degrees
    double prevErr = 0.0;
    uint32_t startMillis = pros::millis();

    while (true) {
        double theta = getHeadingRad();
        double err = targetAngleRad - theta;

        // Normalize to [-pi, pi]
        while (err > M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;

        double deriv = (err - prevErr) / 0.02;
        prevErr = err;

        double out = KP_TURN * err + KD_TURN * deriv;
        out = clamp(out, -2.0, 2.0);

        double k = 60.0;
        double left = clamp(-out * k, -127, 127);
        double right = clamp(out * k, -127, 127);

        robot::drive(left, right);

        if (std::fabs(err * 180.0 / M_PI) < toleranceDeg) break;
        if ((pros::millis() - startMillis) > 5000) break;

        pros::delay(20);
    }

    robot::stop();
}

// ---------------------------
// Move to absolute world coordinates (cm)
// ---------------------------
void pid::moveTo(double targetX, double targetY) {
    const double tolerance = 1.0; // cm
    double prevDistErr = 0.0;
    double prevAngleErr = 0.0;
    uint32_t startMillis = pros::millis();

    while (true) {
        odomMutex.take(); // Lock the mutex
        double curX = odom::getX();
        double curY = odom::getY();
        odomMutex.give(); // Unlock the mutex

        double dx = targetX - curX;
        double dy = targetY - curY;

        double dist = std::hypot(dx, dy);
        double targetAngle = std::atan2(dy, dx);

        // Angle error
        double theta = getHeadingRad();
        double angleErr = targetAngle - theta;
        while (angleErr > M_PI) angleErr -= 2 * M_PI;
        while (angleErr < -M_PI) angleErr += 2 * M_PI;

        // Linear PD
        double distDeriv = (dist - prevDistErr) / 0.02;
        prevDistErr = dist;
        double linearOut = KP_DRIVE * dist + KD_DRIVE * distDeriv;
        linearOut = clamp(linearOut, -120, 120);

        // Angular PD
        double angleDeriv = (angleErr - prevAngleErr) / 0.02;
        prevAngleErr = angleErr;
        double angularOut = KP_TURN * angleErr + KD_TURN * angleDeriv;
        angularOut = clamp(angularOut, -2.0, 2.0);

        double k = 55.0; // scaling factor
        double left = clamp(linearOut - k * angularOut, -127, 127);
        double right = clamp(linearOut + k * angularOut, -127, 127);

        robot::drive(left, right);

        if (dist < tolerance) break;
        if ((pros::millis() - startMillis) > 10000) break;

        pros::delay(20);
    }

    robot::stop();
}

// ---------------------------
// Drive a distance while maintaining a heading
// ---------------------------
void pid::driveToAngle(double distanceCm, double targetAngleRad) {
    const double tolerance = 0.6; // cm
    double prevDistErr = 0.0;
    uint32_t startMillis = pros::millis();

    double startX = odom::getX();
    double startY = odom::getY();

    while (true) {
        // Current position
        double dx = odom::getX() - startX;
        double dy = odom::getY() - startY;
        double traveled = std::hypot(dx, dy);

        // Distance error
        double distErr = distanceCm - traveled;
        double distDeriv = (distErr - prevDistErr) / 0.02;
        prevDistErr = distErr;
        double linearOut = KP_DRIVE * distErr + KD_DRIVE * distDeriv;
        linearOut = clamp(linearOut, -127, 127);

        // Heading error
        double theta = getHeadingRad();
        double headingErr = targetAngleRad - theta;
        while (headingErr > M_PI) headingErr -= 2 * M_PI;
        while (headingErr < -M_PI) headingErr += 2 * M_PI;

        double angularOut = KP_TURN * headingErr;
        angularOut = clamp(angularOut, -20, 20);

        // Drive motors with correction
        robot::drive(linearOut - angularOut, linearOut + angularOut);

        if (std::fabs(distErr) < tolerance) break;
        if ((pros::millis() - startMillis) > 8000) break;

        pros::delay(20);
    }

    robot::stop();
}
