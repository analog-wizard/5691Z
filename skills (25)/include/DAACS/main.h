#ifndef DAACS_MAIN_H
#define DAACS_MAIN_H

#include "vision.h"
#include "Vector/main.h"
#include "Linear/rotation.h"
#include "Linear/direct.h"

class event_loop {
  event_loop() {
    while(true) {};
  };
  ~event_loop();

  
};

#endif