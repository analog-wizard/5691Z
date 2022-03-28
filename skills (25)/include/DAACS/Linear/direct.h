#ifndef DAACS_LINEAR_DIRECT_H
#define DAACS_LINEAR_DIRECT_H

#include <iostream> //For displaying debug information and values via the inbuilt terminal
#include <cmath>    //For caculations and other mathmatical constants
#include "init.h"   //Motors and other devices
#include "PID.h"

x::direct(double x_pos, double y_pos, double power) {
    double delta_x = meas_x - x_pos;
    double delta_y = meas_y - y_pos;

    double dist = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
    double curr_dist += encoder.position(vex::rotationUnits::deg);
    double delta_dist = curr_dist - dist;

    double radian = atan2(delta_y, delta_x);
    double drive;

    this->rotate(100, radian);
    //                                Kp    Kd    Ki
    PID current(0.01, power, -power, 0.01, 0.01, 0.01);
    while(fabs(delta_dist) > 0) {
        curr_dist += encoder.position(vex::rotationUnits::deg);
        delta_dist = curr_dist - dist;

        drive = current.calculate(dist, curr_dist);

        leftFront   .spin(vex::directionType::fwd, -drive, vex::velocityUnits::pct);
        leftMiddle  .spin(vex::directionType::fwd, -drive, vex::velocityUnits::pct);
        leftBack    .spin(vex::directionType::fwd,  drive, vex::velocityUnits::pct);

        rightFront  .spin(vex::directionType::fwd,  drive, vex::velocityUnits::pct);
        rightMiddle .spin(vex::directionType::fwd,  drive, vex::velocityUnits::pct);
        rightBack   .spin(vex::directionType::fwd, -drive, vex::velocityUnits::pct);
    }
}

#endif