#ifndef DAACS_MAIN_H
#define DAACS_MAIN_H

#include "vision.h"
#include <vector>
#include "PID.h"

class event_loop {
  public:
    event_loop() {};

    vex::timer auton_timer = vex::timer();
    vex::thread current_task = vex::thread();

    void main_loop(vex::timer *timer__, std::vector<void*> funct) {
      int pointer_ = 0;
      while(timer__->time() < 15000) {
        vex::thread x((int (*)())(funct[pointer_]));
        x.join();
        pointer_++;
      }
    };
    //~event_loop();
};

class classic : public event_loop {
  friend class field;
  public:
    double kp;
    double ki;
    double kd;

    classic(double kp_, double ki_, double kd_) : event_loop() {
      kp = kp_;
      ki = ki_;
      kd = kd_;
    };
    //~classic();

    void drive(double forward, double speed, double error_margin);
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
    ~field();
};

class vector_based : event_loop {
  public:
    double kp;
    double ki;
    double kd;

    vector_based(double kp_, double ki_, double kd_) : event_loop() {
      kp = kp_;
      ki = ki_;
      kd = kd_;
    };

    typedef struct vector_base {
      double angle_abs;
      double magnitude;
      double x_pos;
      double y_pos;
    } vector_obj;

  void vector_forward(vector_obj v_obj, double power);
};

#endif