#ifndef AUTON_H
#define AUTON_H

#include "init.h"
#include "DAACS/main.h"
#include "DAACS/Linear/direct.h"
#include "DAACS/Vector/main.h"

void autonomous() {
  classic main_drive(0.1, 0.0, 0.0);
  main_drive.drive(1000, 100, 0.1);
  return;
}
#endif