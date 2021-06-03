#ifndef AUTON_H
#define AUTON_H

#include "init.h"
#include "WHEAT.h"

void autonomous() {
  //Current drive measurements are unconfirmed
  robot_ robot (robot_::drive_type::tank_drive, 4.0, 4.0, 200, 0, 0);
  field field_pos(&robot, 0, 0, 0, 0.0);
  
  int8_t motor_num = (int)rightBack.installed() + (int)rightFront.installed() + (int)leftBack.installed() + (int)leftFront.installed()
  + (int)xyroller.installed() + (int)ndroller.installed() + (int)rightIntake.installed() + (int)leftIntake.installed();
  //(Above) count the total number of motors installed in their given ports
  //(Below) if the total number of motors in their ports is less than 8, then do not proceed with Autonomous
  if(motor_num < 8) {
    return; }
  //As per <G9a>: "During the Driver Controlled Period, Drive Team Members may only touch their own Robot if the Robot has not moved at all during the Match.",
  // The goal of stopping the robot from moving during the Autonomous period should a motor be missing is to allow the Drive team to fix the motor as to allow for
  // Standard performance of the driver.
}

#endif
