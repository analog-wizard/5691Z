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

using namespace vex;

void pre_auton ( void ) {
  imu.calibrate();
  vex::task::sleep(3000);

  leftFront.resetRotation();
  leftBack.resetRotation();
  rightFront.resetRotation();
  rightBack.resetRotation();
  rightIntake.resetRotation();
  leftIntake.resetRotation();
  y_encoder.resetRotation();
  x_encoder.resetRotation();
}

void driveCode() {
  leftFront.spin(vex::directionType::fwd,  (controller_.Axis3.value() - (controller_.Axis4.value() * -1)), vex::velocityUnits::pct);
  leftBack.spin(vex::directionType::fwd,   controller_.Axis3.value() - controller_.Axis4.value(),          vex::velocityUnits::pct);
  rightBack.spin(vex::directionType::fwd,  (controller_.Axis2.value() - (controller_.Axis1.value() * -1)), vex::velocityUnits::pct);
  rightFront.spin(vex::directionType::fwd, controller_.Axis2.value() - controller_.Axis1.value(),          vex::velocityUnits::pct);
}

void intakeCode(void) {
  while(true) {
    if(controller_.ButtonR1.pressing()) {
      rightIntake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      leftIntake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      ndroller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else if(controller_.ButtonA.pressing()) {
      leftIntake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
      rightIntake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    }
    else if(controller_.ButtonL1.pressing()) {
      rightIntake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      leftIntake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      ndroller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      xyroller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else if(controller_.ButtonL2.pressing()) {
      rightIntake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
      leftIntake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
      ndroller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
      xyroller.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    }
    else if(controller_.ButtonR2.pressing()) {
      ndroller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      xyroller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else {
      rightIntake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
      leftIntake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
      ndroller.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
      xyroller.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    }
    vex::this_thread::sleep_for(10);
  }
}

void autonmeasure() {
  while(true) {
    if(controller_.ButtonA.pressing()) {
      std::cout << "Linear:\n";
      std::cout << y_encoder.position(vex::rotationUnits::deg) << ":" << ynd_encoder.position(vex::rotationUnits::deg) << ":" << x_encoder.position(vex::rotationUnits::deg) << "\n";
      std::cout << "\n";
    }
    else if(controller_.ButtonY.pressing()) {
      std::cout << "Turn:\n";
      std::cout << rightFront.position(vex::rotationUnits::deg) << ":degree:" << imu.yaw() <<"\n";
      std::cout << "\n";
    }
    else if(controller_.ButtonB.pressing()) {
      x_encoder.resetRotation();
      y_encoder.resetRotation();
      ynd_encoder.resetRotation();
      rightFront.resetRotation();
    }
    vex::this_thread::sleep_for(10);
  }
}

void usercontrol ( void ) {
  //std::cout << vexDeviceMotorFlagsGet(vexDeviceGetByIndex(1)) << std::endl;
  //vexDeviceMotorReverseFlagSet(vexDeviceGetByIndex(1), true);
  //std::cout << vexDeviceMotorFlagsGet(vexDeviceGetByIndex(1)) << std::endl;
  //vexDeviceMotorVelocityPidSet(vexDeviceGetByIndex(1), );
  autonomous();
  //.601/.608
  while(true) {
    vex::thread tdriveCode(driveCode);
    vex::thread tintakeCode(intakeCode);
    vex::thread autonmeasurem(autonmeasure);
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