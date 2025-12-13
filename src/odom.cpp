#include "odom.hpp"
#include "robot.hpp"
#include <cmath>
#include <string>
#include <algorithm>
#include "pros/misc.hpp"

// Sensor objects
pros::Rotation rotHoriz(5); // Horizontal encoder
pros::Rotation rotVert(6);  // Vertical encoder
// pros::Imu imuSensor(21);    // Remove IMU sensor

// Physical constants
static constexpr double WHEEL_DIAMETER_CM = 5.6;
static constexpr double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * M_PI;
static constexpr double CM_PER_DEG = WHEEL_CIRCUMFERENCE_CM / 360.0;

// Robot geometry
static constexpr double ROBOT_WIDTH_CM = 37.0;
static constexpr double ROBOT_LENGTH_CM = 37.0;
static constexpr double ROBOT_CENTER_X = ROBOT_WIDTH_CM / 2.0;
static constexpr double ROBOT_CENTER_Y = ROBOT_LENGTH_CM / 2.0;

// Tracking wheel positions (relative to the bottom-left corner of the robot)
static constexpr double HORIZ_POS_X = 19.0; // Horizontal encoder's x position
static constexpr double HORIZ_POS_Y = 10.5; // Horizontal encoder's y position
static constexpr double VERT_POS_X  = 16.5; // Vertical encoder's x position
static constexpr double VERT_POS_Y  = 15.0; // Vertical encoder's y position

// Internal state
static double globalX = 0.0;
static double globalY = 0.0;
static double thetaDeg = 0.0;

static double prevHorizDeg = 0.0;
static double prevVertDeg  = 0.0;

// Offsets from the robot center
static const double horiz_offset_x = HORIZ_POS_X - ROBOT_CENTER_X; // Adjusted for center
static const double horiz_offset_y = HORIZ_POS_Y - ROBOT_CENTER_Y; // Adjusted for center
static const double vert_offset_x  = VERT_POS_X  - ROBOT_CENTER_X; // Adjusted for center
static const double vert_offset_y  = VERT_POS_Y  - ROBOT_CENTER_Y; // Added for completeness

pros::Mutex odomMutex; // Define the mutex for thread safety

// Deadband and filtering constants
static constexpr double ENCODER_DEADBAND_DEG = 0.1; // Ignore changes smaller than 0.1 degrees
// static constexpr double IMU_DEADBAND_DEG = 0.5;     // Remove IMU deadband constant
static constexpr double MOVEMENT_THRESHOLD_CM = 0.05; // Minimum movement to update odometry

// Utility functions
static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double rad2deg(double r) { return r * 180.0 / M_PI; } // Add rad2deg function

void odom::start() {
    rotVert.reset_position(); // Move this line to the correct location
    prevHorizDeg = rotHoriz.get_position();
    prevVertDeg  = rotVert.get_position();

    static pros::Task odomTask(
        [](void*) {
            while (true) {
                odom::update();
                pros::delay(20); // Ensure the task yields control
            }
        },
        nullptr,
        TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT,
        "odomTask"
    );
}

// Update odometry
void odom::update() {
    odomMutex.take(); // Lock the mutex
    double curH = rotHoriz.get_position();
    double curV = rotVert.get_position();

    double dHdeg = curH - prevHorizDeg;
    double dVdeg = curV - prevVertDeg;
    prevHorizDeg = curH;
    prevVertDeg  = curV;

    double deltaH_cm = dHdeg * CM_PER_DEG;
    double deltaV_cm = dVdeg * CM_PER_DEG;

    double dtheta_rad = 0.0;
    if (fabs(horiz_offset_y) > 1e-6) {
        dtheta_rad = -deltaH_cm / horiz_offset_y;
    }

    double deltaY_local = deltaV_cm - vert_offset_x * dtheta_rad;
    double deltaX_local = deltaH_cm;

    double theta_mid = deg2rad(thetaDeg) + dtheta_rad * 0.5;
    double cosT = cos(theta_mid);
    double sinT = sin(theta_mid);

    double dX_world = deltaX_local * cosT - deltaY_local * sinT;
    double dY_world = deltaX_local * sinT + deltaY_local * cosT;

    globalX += dX_world;
    globalY += dY_world;

    thetaDeg += rad2deg(dtheta_rad);
    while (thetaDeg >= 180.0) thetaDeg -= 360.0;
    while (thetaDeg < -180.0) thetaDeg += 360.0;

    // Debug print
    pros::lcd::print(7, "X: %.2f, Y: %.2f, Theta: %.2f", globalX, globalY, thetaDeg);

    odomMutex.give(); // Unlock the mutex
}

// Getters
double odom::getX() { return globalX; }
double odom::getY() { return globalY; }
double odom::getTheta() { return thetaDeg; }
double odom::getXmm() { return globalX * 10.0; } // Convert to millimeters
double odom::getYmm() { return globalY * 10.0; } // Convert to millimeters
double odom::getThetaDisplacement() { return thetaDeg; } // Theta displacement from start

void odom::setPosition(double x, double y, double theta) {
    odomMutex.take(); // Lock the mutex
    globalX = x;
    globalY = y;
    thetaDeg = theta;
    odomMutex.give(); // Unlock the mutex
}

void odom::getPosition(double& x, double& y, double& theta) {
    odomMutex.take();
    x = globalX;
    y = globalY;
    theta = thetaDeg;
    odomMutex.give();
}

void odom::reset() {
    odomMutex.take(); // Lock the mutex
    globalX = 0.0;
    globalY = 0.0;
    thetaDeg = 0.0;
    prevHorizDeg = rotHoriz.get_position();
    prevVertDeg = rotVert.get_position();
    odomMutex.give(); // Unlock the mutex
}