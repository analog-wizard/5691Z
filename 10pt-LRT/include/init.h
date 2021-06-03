#ifndef INIT_H
#define INIT_H

vex::brain brain_;
vex::competition competition_;
vex::controller controller_;

vex::motor leftFront = vex::motor(vex::PORT16, vex::gearSetting::ratio18_1, false);
vex::motor leftBack = vex::motor(vex::PORT17, vex::gearSetting::ratio18_1, false);
vex::motor rightFront = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor rightBack = vex::motor(vex::PORT5, vex::gearSetting::ratio18_1, true);
//The only agreed upon aspect (currently) is a Double Reverse 4 Bar (DR4B)
vex::motor dr4b = vex::motor(vex::PORT6, vex::gearSetting::ratio36_1, false);

//An 8 long array used to store port numbers for installation checks
// (To see if the motor is in the port specified)
int8_t port_array[8] = {16, 17, 2, 5, 9, 14, 15, 10};

//vex::bumper front_limit = vex::bumper(brain_.ThreeWirePort.H);
vex::inertial imu = vex::inertial(vex::PORT18);

vex::encoder y_left_encoder = vex::encoder(brain_.ThreeWirePort.A);
vex::encoder y_right_encoder = vex::encoder(brain_.ThreeWirePort.C);

#endif
