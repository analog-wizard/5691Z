#ifndef DAACS_DIRECT_H
#define DAACS_DIRECT_H

#include "init.h"
#include "DAACS/main.h"

void classic::drive(double forward, double speed, double error_margin) {
  double delta_pos;

  double drive_power;

  double proc_var = 0;
  PID power(speed, -speed, 0.1, 0.0, 0.0);

  double time_prev;
  double delta_time = 0;

  while(fabs(delta_pos) > error_margin) {
    time_prev = vex::timer::systemHighResolution();
    drive_power = power.calculate( delta_time, forward, proc_var);
    std::cout << drive_power << "\n";

    rightBack.spin(vex::directionType::fwd,  drive_power, vex::velocityUnits::pct);
    rightMiddle.spin(vex::directionType::fwd,drive_power, vex::velocityUnits::pct);
    rightFront.spin(vex::directionType::fwd, drive_power, vex::velocityUnits::pct);

    leftBack.spin(vex::directionType::fwd,  drive_power, vex::velocityUnits::pct);
    leftMiddle.spin(vex::directionType::fwd,drive_power, vex::velocityUnits::pct);
    leftFront.spin(vex::directionType::fwd, drive_power, vex::velocityUnits::pct);

    proc_var = x_encoder.position(vex::rotationUnits::deg);
    delta_pos = forward - proc_var;
    vex::task::sleep(10);
    delta_time = time_prev - vex::timer::systemHighResolution();
  }
  rightBack.stop();
  rightMiddle.stop();
  rightFront.stop();

  leftBack.stop();
  leftMiddle.stop();
  leftFront.stop();
}

#endif