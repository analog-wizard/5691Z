#ifndef AUTON_H
#define AUTON_H

#include "init.h"
#include "WHEAT.h"
//*
void autonomous() {
  bool work = true;
  std::cout << work << "\n";
  drivetrain_ _drivetrain(drivetrain_::drive_type::tank_drive, &x_encoder, nullptr, &xnd_encoder, nullptr, &imu);
  //In CM
  robot_ robot(10.16, 6.985, 257.14, 0, 0, &_drivetrain);
  field field_(&robot, 0, 0, 0, 0);
  field_.start_yaw = field_.robot_p->drive.obj->pos_devices.imu_->heading(vex::rotationUnits::deg);
  field_.pos.yaw = 0;

  std::cout << field_.pos.yaw << "\n";
  double delta_rotation = (45 - field_.pos.yaw);
  double prof = ((100*2)/(1+(pow(2.718281828459045235,0.025641 * delta_rotation)))-100);
  std::cout << delta_rotation << ":" << prof << ":" << "\n";
  while(fabs(delta_rotation) > 0) {
    delta_rotation = (field_.pos.yaw - 45);
    prof = ((100*2)/(1+(pow(2.718281828459045235,0.025641 * delta_rotation)))-100);
    std::cout << delta_rotation << ":" << prof << ":" << "\n";
    /*
    leftFront.  spin(vex::directionType::fwd, -prof, vex::percentUnits::pct);
    leftMiddle. spin(vex::directionType::fwd, -prof, vex::percentUnits::pct);
    leftBack.   spin(vex::directionType::fwd, prof, vex::percentUnits::pct);

    rightFront. spin(vex::directionType::fwd, prof, vex::percentUnits::pct);
    rightMiddle.spin(vex::directionType::fwd, prof, vex::percentUnits::pct);
    rightBack.  spin(vex::directionType::fwd, -prof, vex::percentUnits::pct);*/
  }

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

//field_.direct(50, field::units::raw, 300, 0, 5, vex::directionType::fwd);
//                                    \/ Unknown
field_.direct(50, field::units::raw, 1000, -300, 5, vex::directionType::fwd);
//field_.rotate(100, 45, 0);
return;
field_.rotate(100, 45, 0);
//                                    \/ Unknown
field_.direct(100, field::units::raw, 0,0, 0, vex::directionType::fwd);
//Rotate to set back mogo lift to red goal
field_.rotate(100, 180, 0);
mogo.rotateTo(0, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);
//                                    \/ Unknown
field_.direct(100, field::units::raw, 0,0, 0, vex::directionType::fwd);
//Lift goal 45 up
mogo.rotateTo(1000, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);
//Rotate to center mobile goal
field_.rotate(100, 90, 0);
//Forward to grab goal
field_.direct(100, field::units::raw, 0,0, 0, vex::directionType::fwd);
hook.set(true);
//Backwards
field_.direct(100, field::units::raw, 0,0, 0, vex::directionType::fwd);



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