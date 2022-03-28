#ifndef DAACS_MAIN_H
#define DAACS_MAIN_H

#include "vision.h"
#include <vector>
#include "Vector/main.h"
#include "Linear/rotation.h"
#include "Linear/direct.h"

class event_loop {
  public:
    event_loop() {};
    ~event_loop();
  
};

class field : public event_loop {
  public:
    typedef struct angle_3d {
      double yaw;
      double pitch;
      double roll;
    } angles;
    angles angle_3d;
    
    typedef struct game_obj {
      double yaw;
      double pitch;
      double roll;
    } element_obj;

    double x_pos;
    double y_pos;
    std::vector<game_obj> obj_vect;

    field(double x_pos, double y_pos, angles angle_, std::vector<game_obj> obj_vect_) : event_loop() {
      x_pos = x_pos;
      y_pos = y_pos;
      field::angle_3d = angle_;
      obj_vect = obj_vect_;
    };
};

#endif