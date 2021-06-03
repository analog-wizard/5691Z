#ifndef AUTON_H
#define AUTON_H

#include "init.h"
#include "WHEAT.h"

void autonomous() {
  //Current drive measurements are unconfirmed
  robot_ robot (robot_::drive_type::tank_drive, 4.0, 4.0, 200, 0, 0);
  field field_pos(&robot, 0, 0, 0, 0.0);
}

#endif
