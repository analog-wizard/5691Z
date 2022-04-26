#ifndef DAACS_MAIN_H
#define DAACS_MAIN_H

#include "vision.h"

#include <vector>
#include <atomic>
#include <chrono>

class event_loop {
  private:
    vex::inertial* imu;
    vex::vision* vision;
  public:
    event_loop(vex::inertial* imu_) {
        imu = imu_
    };

    vex::timer auton_timer = vex::timer();
    std::promise<bool> thread_complete;
    vex::thread current_task;

    typedef struct collision {
      double yaw;
      double magnitude;
    } _collision;
    _collision collision;

    std::atomic<std::vector<void*>> event_bus;
    event_bus.reserve(2);

    void main_loop(vex::timer *timer__, std::vector<void*> funct) {
      using namespace std::chrono_literals;
      int pointer_ = 0;
      while(timer__->time() < 15000) {
        vex::thread current_task((void (*)())(funct[pointer_]));
        auto future = this->thread_complete.get_future();
        auto status = future.wait_for(0ms);
        double prev_left_accel;
        double prev_right_accel;
        double prev_right_velocity = 0;
        double prev_left_velocity = 0;
        while(status != std::future_status::ready) {
            status = future.wait_for(0ms);
            if(imu->installed()) {
                prev_left_accel = (((leftBack.velocity()+leftMiddle.velocity()+leftFront.velocity())/3)-prev_left_velocity)/delta_time:
                prev_right_accel = (((rightBack.velocity()+rightMiddle.velocity()+rightFront.velocity())/3)-prev_right_velocity)/delta_time;
                imu->acceleration(vex::axisType::xaxis);
                imu->acceleration(vex::axisType::yaxis);
            }
            if(vision->installed()) {
                
            }
        }
        pointer_++;
      }
      return;
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
    std::atomic<std::vector<game_obj>> obj_vect;

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
