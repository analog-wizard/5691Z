#ifndef INIT_H
#define INIT_H

vex::brain brain_;

vex::controller controller_;

vex::motor leftFront = vex::motor(vex::PORT16, vex::gearSetting::ratio18_1, false);
vex::motor leftBack = vex::motor(vex::PORT17, vex::gearSetting::ratio18_1, false);
vex::motor rightFront = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor rightBack = vex::motor(vex::PORT5, vex::gearSetting::ratio18_1, true);
vex::motor rightIntake = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor leftIntake = vex::motor(vex::PORT14, vex::gearSetting::ratio18_1, false);
vex::motor ndroller = vex::motor(vex::PORT15, vex::gearSetting::ratio18_1, false);
vex::motor xyroller = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, false);

vex::competition competition_;

vex::bumper front_limit = vex::bumper(brain_.ThreeWirePort.H);

vex::inertial imu = vex::inertial(vex::PORT18);

vex::encoder y_encoder = vex::encoder(brain_.ThreeWirePort.A);
vex::encoder ynd_encoder = vex::encoder(brain_.ThreeWirePort.C);
vex::encoder x_encoder = vex::encoder(brain_.ThreeWirePort.E);

#endif