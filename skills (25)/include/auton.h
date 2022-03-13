#ifndef AUTON_H
#define AUTON_H

#include "init.h"
#include "DAACS/main.h"

void drive(double left, double right, double speed) {
  rightBack.rotateFor(vex::directionType::fwd, right, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
  rightMiddle.rotateFor(vex::directionType::fwd, right, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
  rightFront.rotateFor(vex::directionType::fwd, right, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);

  rightBack.rotateFor(vex::directionType::fwd, right, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
  rightBack.rotateFor(vex::directionType::fwd, right, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
  rightBack.rotateFor(vex::directionType::fwd, right, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
}

void autonomous() {

//
//Auto 1 "Point Auto"
  //Forward
  //Grab front red mogo
  //Front turn right 90
  /*Back up so center align with bottom most yellow mogo (AV)
      & lower back mogo lift*/

  //Front turn right 90
  //Back up
  //Raise back mogo lift

//claw.set(true);
//vex::task::sleep(750);
//claw.set(false);
//vex::task::sleep(1000);
//claw.set(true);

//robot.direct(100, robot_::units::raw, 100, 0, vex::directionType::fwd, 20);



//Left distance, Right distance, Power
/*
//Move backward to pick up alliance goal
drive(-450, -450, 100);
hook.set(true);
hook_2.set(true);

//Turn so that forward faces the far center goal (audience)
drive(-1755, 1755, 100);
claw.set(false);

lift.rotateTo(250, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
conveyer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
//Go forward to grab far center goal (audience)
drive(2664, 2664, 100);
lift.rotateTo(0, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
conveyer.stop(vex::brakeType::coast);
claw.set(true);

//lift up
lift.startRotateTo(574, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
//Go forward to the platform
conveyer.stop(vex::brakeType::coast);
drive(4020, 4020, 50);

//lift down, release and back away from the platform
claw.set(false);
drive(-300, -300, 100);

//start lift down
lift.startRotateTo(0, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);

//turn to face center goal then go towards
drive(-1020, 1020, 100);
drive(1545, 1545, 100);

//Grab middle goal and then turn back to the platform
claw.set(true);
drive(1458, -1458, 100);
//start lift up
lift.startRotateTo(574, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);

//Go forward to platform then drop, then back up
drive(1398, 1398, 100);
claw.set(false);
drive(-300, -300, 100);

//Turn to get close middle goal (audience) then turn to platform
lift.startRotateTo(0, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
drive(-1052, 1052, 100);

//forward to plaform
drive(2106, 2106, 100);
//turn to get 
drive(1560, -1560, 100);
drive(2445, 2445, 100);
drive(-594, -594, 100);

hook.set(true);
hook_2.set(true); 

drive(141, -141, 100);
drive(-805, -805, 100);
//lift down
*/
/*MEASURED FROM BACK LEFT
-150 back
clamp
-585 left turn
claw
888 forward
lift up
1340 forward
lift down
-100 back
-340 turn left and lift down
515 forward
clamp
486 turn right
466 forward and lift down
-100 back
-352 left turn and lift down more
702 forward
520 right and lift up
815 forward 
-198 back
clamp
141 right turn
-805 back
lift down 
*/

//field_.direct(100, field::units::raw, 200, 0, 1, vex::directionType::fwd, 5);

//field_.rotate(50, 270, 0.5);
//field_.direct(100, field::units::raw, 500, 0, 0.1, vex::directionType::rev, 7)
return;

//Auto 2 "WP Auto"
  //Forward
  //Drop rings via top claw (??) & grab mogo
  //Go forward so center is on +1 tile
  //Front turn right 90
  /*"Backward" to the other side of the platform &
      & 25degrees (from AV right) center robot*/
  //Front turn 115 left
  //Deposite back rings on back mogo lift onto goal

  vex::timer __timer__;
}
#endif