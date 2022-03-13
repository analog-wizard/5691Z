#ifndef USER_OPP_H
#define USER_OPP_H

#include "init.h"
bool grasp = false;
bool under_hook = false;
bool under_hook_2 = false;

void pnue_() {
  if(controller_.ButtonA.pressing()) {
    grasp = !grasp;
    claw.set(grasp);
    vex::task::sleep(125);
  }
  if(controller_.ButtonB.pressing()) {
    under_hook = !under_hook;
    under_hook_2 = !under_hook_2;
    hook.set(under_hook);
    hook_2.set(under_hook_2);
    vex::task::sleep(125);
  }
}

void lift_grasp() {
  while(true) {
    lift.spin(vex::directionType::fwd, ((int)controller_.ButtonL1.pressing() * 100) + ((int)controller_.ButtonL2.pressing() * -100), vex::velocityUnits::pct);
    conveyer.spin(vex::directionType::fwd, ((int)controller_.ButtonR2.pressing() * 80) + ((int)controller_.ButtonR1.pressing() * -80), vex::velocityUnits::pct);
    vex::task::sleep(10);
  }
}

void driveCode() {
  while(true) {
    leftFront. spin(vex::directionType::fwd, -controller_.Axis3.position(), vex::velocityUnits::pct);
    leftMiddle.spin(vex::directionType::fwd, -controller_.Axis3.position(), vex::velocityUnits::pct);
    leftBack. spin(vex::directionType::fwd, controller_.Axis3.position(), vex::velocityUnits::pct);

    rightFront. spin(vex::directionType::fwd, controller_.Axis2.position(), vex::velocityUnits::pct);
    rightMiddle.spin(vex::directionType::fwd, controller_.Axis2.position(), vex::velocityUnits::pct);
    rightBack. spin(vex::directionType::fwd, -controller_.Axis2.position(), vex::velocityUnits::pct);
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