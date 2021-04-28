#ifndef AUTON_H
#define AUTON_H

#include "init.h"
#include "WHEAT.h"

void autonomous() {
  //                                                          Size for later
  //                                                          \/
  robot_ robot (robot_::drive_type::x_drive, 3.25, 2.75, 200, 0, 0);
  field field_pos(&robot, 0, 0, 0, 0.0);

  field_pos.arc(50, field::units::raw, 159, 0, 0, 0.855, 0.254);
}

#endif