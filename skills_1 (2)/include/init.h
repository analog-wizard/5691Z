#ifndef INIT_H
#define INIT_H

#include <array>

vex::brain brain_;
vex::controller controller_;

vex::motor leftFront =    vex::motor(vex::PORT8, vex::gearSetting::ratio6_1, false);
vex::motor leftMiddle =   vex::motor(vex::PORT4,  vex::gearSetting::ratio6_1, true);
vex::motor leftBack =     vex::motor(vex::PORT16,  vex::gearSetting::ratio6_1, false); //3 

vex::motor rightFront =   vex::motor(vex::PORT5, vex::gearSetting::ratio6_1, true);
vex::motor rightMiddle =  vex::motor(vex::PORT7,  vex::gearSetting::ratio6_1, false);
vex::motor rightBack =    vex::motor(vex::PORT13,  vex::gearSetting::ratio6_1, true);

vex::motor_group left_drive = vex::motor_group(leftFront, leftMiddle, leftBack);
vex::motor_group right_drive = vex::motor_group(rightFront, rightMiddle, rightBack);

vex::motor lift =         vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor conveyer =     vex::motor(vex::PORT6, vex::gearSetting::ratio36_1, false);

vex::inertial imu =       vex::inertial(vex::PORT1);

vex::digital_out claw =   vex::digital_out(brain_.ThreeWirePort.A);
vex::digital_out hook =   vex::digital_out(brain_.ThreeWirePort.B);
vex::digital_out hook_2 = vex::digital_out(brain_.ThreeWirePort.C);
vex::encoder x_encoder =  vex::encoder(brain_.ThreeWirePort.G);

std::array<uint8_t,21> device_array;
std::array<uint8_t,9> port_discov;
std::array<uint8_t,9> port_array = {1, 2, 4, 5, 6, 7, 13, 16, 18};

vex::devices device_proxy = vex::devices();
bool device_check(const uint8_t motor_num) {
  //std::cout << device_proxy.type(4) << "\n";
  //std::cout << device_proxy.numberOf((V5_DeviceType)2) << "\n";

  for(uint8_t i; i<21; i++) {
    uint8_t x = 0;
    if((int)vexDeviceGetByIndex(i) != 0) {
      port_discov[x] = (int)vexDeviceGetByIndex(i);
      x++;
    } else;
  }

  bool check_pass = false;
  //switch(device_proxy.numberOf((V5_DeviceType)2)) { //motor
  //switch(device_proxy.numberOf((V5_DeviceType)6)) { //imu
  return check_pass;
}

vex::competition competition_;
#endif