/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ian                                                       */
/*    Created:      Sat Nov 07 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "init.h"
#include "auton.h"
#include "user_opp.h"

using namespace vex;

void pre_auton ( void ) {
  //device_vector.reserve(21);
  //port_discov.reserve(21);
  //device_check();

  leftFront.resetRotation();
  leftMiddle.resetRotation();
  leftBack.resetRotation();
  rightFront.resetRotation();
  rightMiddle.resetRotation();
  rightBack.resetRotation();

  x_encoder.resetRotation();
  xnd_encoder.resetRotation();

  mogo.stop(vex::brakeType::hold);

  //device_check(check_list);
/*
  for(int8_t i = 0; i++; i<=8) {
    controller_.Screen.setCursor(3, i);
    if (vex::motor((port_array[i] - 1)).installed())
      controller_.Screen.print("1");
    else
      controller_.Screen.print("0");
  }
*/
/*
  imu.calibrate();
  vex::task::sleep(3000);
  while(imu.isCalibrating()) {}
  std::cout << "Imu Finished" << "\n"
*/
}

void usercontrol ( void ) {
  //pre_auton();
  autonomous();
  vex::timer _timer;

  leftFront.stop(vex::brakeType::coast);
  leftMiddle.stop(vex::brakeType::coast);
  leftBack.stop(vex::brakeType::coast);
  rightFront.stop(vex::brakeType::coast);
  rightMiddle.stop(vex::brakeType::coast);
  rightBack.stop(vex::brakeType::coast);
  while(true) {
    vex::thread tdriveCode(driveCode);
    vex::thread tlift(lift_grasp);
    pnue_();
    //vibrate_time(&_timer);
    vex::task::sleep(10);
  }
}

int main() {
  competition_.autonomous( autonomous );
  competition_.drivercontrol( usercontrol );

  pre_auton();

  while(1) {
    vex::task::sleep(100);
  }
}