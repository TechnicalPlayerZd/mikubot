#pragma once
#include "api.h"

namespace odom {

void start();
void reset(); // Declare reset function
void update();
double getX();
double getY();
double getTheta();
double getXmm(); // Declare getXmm
double getYmm(); // Declare getYmm
double getThetaDisplacement();
void setPosition(double x, double y, double theta); // Declare setPosition function
void getPosition(double& x, double& y, double& theta);

} // namespace odom

extern pros::Mutex odomMutex; // Declare the mutex as extern