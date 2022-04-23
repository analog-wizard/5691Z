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
#include <iostream>

using namespace vex;

void pre_auton ( void ) {
  leftFront.resetRotation();
  leftMiddle.resetRotation();
  leftBack.resetRotation();
  rightFront.resetRotation();
  rightMiddle.resetRotation();
  rightBack.resetRotation();

  conveyer.stop(vex::brakeType::hold);
  lift.stop(vex::brakeType::hold);
}

void usercontrol ( void ) {
  //autonomous();
  vex::timer _timer;

  lift.stop(vex::brakeType::hold);
  conveyer.stop(vex::brakeType::hold);

  leftFront.stop(vex::brakeType::coast);
  leftMiddle.stop(vex::brakeType::coast);
  leftBack.stop(vex::brakeType::coast);
  rightFront.stop(vex::brakeType::coast);
  rightMiddle.stop(vex::brakeType::coast);
  rightBack.stop(vex::brakeType::coast);
  while(true) {
    //std::cout << brain_.Screen.xPosition() << brain_.Screen.yPosition() << "\n";
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