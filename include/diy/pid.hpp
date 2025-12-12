#pragma once
#include "pros/apix.h" // Replace "main.hpp" with the correct PROS API header

namespace pid {

double compute(double target, double current, double kP, double kI, double kD);

void driveTo(double targetDistance); // relative distance in cm
void turnTo(double targetAngleRad);  // absolute heading in radians
void moveTo(double targetX, double targetY); // world coordinates
void driveToAngle(double distanceCm, double targetAngleRad);

} // namespace pid

class PID {
public:
  PID(double kP, double kI, double kD);

  void setTarget(double target);
  double calculate(double current);
  void reset();

private:
  double kP, kI, kD;
  double target;
  double error, prevError, integral;
};
