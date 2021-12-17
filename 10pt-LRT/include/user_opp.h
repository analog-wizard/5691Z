#ifndef USER_OPP_H
#define USER_OPP_H

#include "init.h"
bool grasp = false;
bool under_hook = false;

void pnue_() {
  if(controller_.ButtonA.pressing()) {
    grasp = !grasp;
    claw.set(grasp);
    vex::task::sleep(125);
  }
  if(controller_.ButtonLeft.pressing()) {
    under_hook = !under_hook;
    hook.set(under_hook);
    vex::task::sleep(125);
  }
}

void lift_grasp() {
  while(true) {
    if(lift.rotation(vex::rotationUnits::deg) >= 1705) {
      lift.spin(vex::directionType::fwd, ((int)controller_.ButtonL2.pressing() * -100), vex::velocityUnits::pct);
    } else if(lift.rotation(vex::rotationUnits::deg) <= 20) {
      lift.spin(vex::directionType::fwd, ((int)controller_.ButtonL1.pressing() * 100), vex::velocityUnits::pct);
    } else {
      lift.spin(vex::directionType::fwd, ((int)controller_.ButtonL1.pressing() * 100) + ((int)controller_.ButtonL2.pressing() * -100), vex::velocityUnits::pct);
    }

    if(mogo.rotation(vex::rotationUnits::deg) >= 1705) {
      mogo.spin(vex::directionType::fwd, ((int)controller_.ButtonR1.pressing() * -100), vex::velocityUnits::pct);
    } else if(mogo.rotation(vex::rotationUnits::deg) <= 0) {
      mogo.spin(vex::directionType::fwd, ((int)controller_.ButtonR2.pressing() * 100), vex::velocityUnits::pct);
    } else {
      mogo.spin(vex::directionType::fwd, ((int)controller_.ButtonR2.pressing() * 100) + ((int)controller_.ButtonR1.pressing() * -100), vex::velocityUnits::pct);
    }
  }
}

bool drive_slow = false;
void driveCode() {
  //void* x is a void pointer (a generic pointer that is non data-type specific)
  //this variable currently holds the status of of a boolean named "platform_speed",
  //which determines if the robot is on the platform and the speed should be reduced
  //
  //Given that boolean values can act as a integer value when type-casted/converted
  //To reduce the speed by half, all that is needed is to multiply the boolean (either a 1 or 0) by 2
  //When "true" (1), the value of the controller axes will be divided by 2, else, the value will not be divided
  while(true) {
    if(controller_.ButtonY.pressing()) {
      std::cout << drive_slow << "\n";
      drive_slow = !drive_slow;
      vex::task::sleep(175);}
    double eval_power = (drive_slow ? 0.6 : 1);
    leftFront. spin(vex::directionType::fwd, -controller_.Axis3.position() * eval_power, vex::velocityUnits::pct);
    leftMiddle.spin(vex::directionType::fwd, -controller_.Axis3.position() * eval_power, vex::velocityUnits::pct);
    leftFront. spin(vex::directionType::fwd, controller_.Axis3.position() * eval_power, vex::velocityUnits::pct);

    rightFront. spin(vex::directionType::fwd, controller_.Axis2.position() * eval_power, vex::velocityUnits::pct);
    rightMiddle.spin(vex::directionType::fwd, controller_.Axis2.position() * eval_power, vex::velocityUnits::pct);
    rightFront. spin(vex::directionType::fwd, -controller_.Axis2.position() * eval_power, vex::velocityUnits::pct);
    vex::task::sleep(10);
  }
}

void vibrate_time(vex::timer *timer__) {
  if(((int)timer__->time() % 10000) == 0 && !(timer__->time() == 75000) && !(timer__->time() >= 90000)) {
    controller_.rumble("-.");
  } else if((timer__->time() == 75000)) {
    controller_.rumble("..-.");
  } else if(((int)timer__->time() % 1000) == 0 && (timer__->time() > 75000) && !(timer__->time() <= 90000)) {
    controller_.rumble(".");
  } else if((int)timer__->time() > 105000) {} else;
}

#endif