#ifndef WHEAT_H
#define WHEAT_H

#include <cmath>
#include <iostream>
#include "init.h"

/*
Author:         Alexander Spicer
Creation Date:  ~APR/14/2020
Purpose:        To move the robot in a coordinated and modular manner
*/

class drivetrain_ {
  public:
    enum drive_type {
      tank_drive = 1,
      x_drive = 2,
      mecanum = 3,
      h_drive = 4,
      dragon_drive = 5,
      swerve = 6
    };

    drivetrain_::drive_type drive_;

    struct pos_devices{
      vex::encoder* x_enc;
      vex::encoder* y_enc;
      vex::encoder* y_enc_nd;
      vex::encoder* x_enc_nd;
      vex::inertial* imu_;
    } pos_devices;

    drivetrain_(drivetrain_::drive_type drive__, void* x_enc_, void* y_enc_, void* x_enc_nd, void* y_enc_nd , void* _imu_) {
      if(x_enc_)
        pos_devices.x_enc = (vex::encoder*)x_enc_;
      else;

      if(y_enc_)
        pos_devices.y_enc = (vex::encoder*)y_enc_;
      else;

      if(x_enc_nd)
        pos_devices.x_enc_nd = (vex::encoder*)x_enc_nd;
      else;

      if(y_enc_nd)
        pos_devices.y_enc_nd = (vex::encoder*)y_enc_nd;
      else;
      
      if(_imu_)
        pos_devices.imu_ = (vex::inertial*)_imu_;
      else;
      drive_ = drive__;
    }
    ~drivetrain_() {}
};

class robot_ {
  public:
    struct drive {
      //In inches..
      double wh_size;
      //The most commmon design for tracking wheels requires the 2.75 inch Omni wheel, so 2.75 is the default
      double tr_wh_size = 2.75;
      //In (Driven wheel) RPM..
      double speed;
      //In inches..
      double x_size;
      double y_size;
      drivetrain_* obj;
    } drive;

    //Constructor
    robot_(double wh_size, double tr_wh_size, double speed, double x_size, double y_size, drivetrain_* obj) {
      robot_::drive.obj = obj;
      robot_::drive.wh_size = wh_size;
      robot_::drive.tr_wh_size = tr_wh_size;
      robot_::drive.speed = speed;
      robot_::drive.x_size = x_size;
      robot_::drive.y_size = y_size;
    }
    robot_(double wh_size, double speed, double x_size, double y_size, drivetrain_* obj_) {
      robot_::drive.obj = obj_;
      robot_::drive.wh_size = wh_size;
      robot_::drive.speed = speed;
      robot_::drive.x_size = x_size;
      robot_::drive.y_size = y_size;
    }
    //Destructor
    ~robot_() {};
};

class field {
  friend class platform;
    public:
    //Pointer to the robot class instance passed at field class construction
    robot_ *robot_p;

    //Positional data
    double start_yaw;
    struct pos_ {
      int32_t x_pos;
      int32_t y_pos;
      //Z axis is for use in high hangs or other platform based mechanics
      double z_pos;
      double pitch;
      double yaw;
      double roll;
    } pos;
    //Eular's number, which is used in the Sigmoid curve motion profiling seen throughout the library
    const double eular = 2.718281828459045235;
    //Units used to determine in what increments the robot can move
    enum units {
      tile = 1000,
      meter = 250,
      foot = 100,
      inches = 10,
      raw = 1
    };
    //Constructor
    //*robot_1 is a "robot" class pointer used to refer to the physical aspects of the robot
    //*robot_p is always initialized in order to avoid a Segmentation fault, which can result from accessing a uninitialized pointer
    field(robot_ *robot_1, int32_t x_pos, int32_t y_pos, double z_pos, double yaw) {
      //robot_p is assigned the value of robot_1 to allow access to the passed robot instance to all members of the "field" class
      robot_p = robot_1;
      pos.x_pos = x_pos; 
      pos.y_pos = y_pos; 
      pos.z_pos = z_pos;
      pos.yaw = yaw;
    };
    //Destructor
    ~field() {};
    
    //Used to print the position of the robot in an (x,y,z,angle) manner
    void display    (); 
    //Used to set the position of the robot in an (x,y,z,angle) manner
    void set_pos    (field::units units, int32_t x_pos, int32_t y_pos, double z_pos);

    //Used to rotate the robot to a desired angle at a desired power
    void rotate     (uint8_t power, double rotation, double margin); 

    //To move in a straight line between the current position and a given point at a given speed
    //The "bool button" is used to enable barrier sensing, which would stop the robot should a obstacle was encountered
    void direct     (uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, vex::directionType direction);

    //To move in a straight line between the current position and a given point at a given speed within a given time limit
    //The "bool button" is used to enable barrier sensing, which would stop the robot should a obstacle was encountered
    void direct_time(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, uint16_t time);
};

class platform {
  protected:
    //Pointer to the main field instance
    field *main_field;
    //Represents the position of the robot in a relative coordinate grid of the platform in an (X,Y,Z,yaw,roll,pitch) form
    struct rel_pos {
      int32_t x_pos;
      int32_t y_pos;
      //Z axis is for use in high hangs or other platform based mechanics
      double z_pos;
    } rel_pos;
    struct plat_angle {
      //Angles of the platform are represented as a (yaw,roll,pitch)
      double yaw;
      double roll;
      double pitch;
    } plat_angle;
    //X and Y limit of the platform
    int32_t x_limit;
    int32_t y_limit;
  public:
    //Constructor
    platform(int32_t x_pos, int32_t y_pos, double z_pos, double yaw, double roll, double pitch, int32_t x_limit, int32_t y_limit, field *field_obj) {
      rel_pos.x_pos = x_pos;
      rel_pos.y_pos = y_pos;
      rel_pos.z_pos = z_pos;
      plat_angle.yaw = yaw;
      plat_angle.roll = roll;
      plat_angle.pitch = pitch;
      platform::x_limit = x_limit;
      platform::y_limit = y_limit;
      main_field = field_obj;
    };
    //Destructor
    ~platform() {};
    
    //prints value(s) of the current (X,Y,Z,angle) data
    void display_();
    //sets the position
    void tare();
};

void field::display() {
  std::cout << pos.x_pos << ": " << pos.y_pos << ": " << pos.z_pos << ": " << pos.yaw << "\n";
}

//47.9 cm diameter between both sides of drive wheels
//32.4 cm diameter between tracking wheels
void field::rotate(uint8_t power, double rotation, double margin) { //Make rotate direction / shortest rotational distance
  std::cout << this->pos.yaw << "\n";
  double delta_rotation = (rotation - this->pos.yaw);
  double prof = ((power*2)/(1+(pow(eular,0.025641 * delta_rotation)))-power);
  std::cout << delta_rotation << ":" << prof << ":" << "\n";
  while(fabs(delta_rotation) > margin) {
    delta_rotation = (this->pos.yaw - rotation);
    prof = ((power*2)/(1+(pow(eular,0.025641 * delta_rotation)))-power);
    std::cout << delta_rotation << ":" << prof << ":" << "\n";
    /*
    leftFront.  spin(vex::directionType::fwd, -prof, vex::percentUnits::pct);
    leftMiddle. spin(vex::directionType::fwd, -prof, vex::percentUnits::pct);
    leftBack.   spin(vex::directionType::fwd, prof, vex::percentUnits::pct);

    rightFront. spin(vex::directionType::fwd, prof, vex::percentUnits::pct);
    rightMiddle.spin(vex::directionType::fwd, prof, vex::percentUnits::pct);
    rightBack.  spin(vex::directionType::fwd, -prof, vex::percentUnits::pct);*/
  }
};

void field::direct(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, vex::directionType direction) { //Add direction
  double d_x_pos = (pos.x_pos-x_pos);
  double d_y_pos = (pos.y_pos-y_pos);

  long double radian = atan2(y_pos, x_pos);
  field::rotate(power, radian, error_margin);
  long double distancew;

  if(direction == vex::directionType::fwd)
    distancew = -(int)sqrt(pow((d_x_pos),2.00) + pow((d_y_pos),2.00));
  else if(direction == vex::directionType::rev)
    distancew = (int)sqrt(pow((d_x_pos),2.00) + pow((d_y_pos),2.00));
  else;
  long double left_x_d = distancew - this->robot_p->drive.obj->pos_devices.x_enc->position(vex::rotationUnits::deg);
  long double right_xnd_d = distancew - this->robot_p->drive.obj->pos_devices.x_enc_nd->position(vex::rotationUnits::deg);

  double side_left_prof;
  double side_right_prof;
  while(fabs(left_x_d) > error_margin || fabs(right_xnd_d) > error_margin) {
    left_x_d = distancew - this->robot_p->drive.obj->pos_devices.x_enc->position(vex::rotationUnits::deg);
    right_xnd_d = distancew + this->robot_p->drive.obj->pos_devices.x_enc_nd->position(vex::rotationUnits::deg);

    if(left_x_d > 0) {
      side_left_prof = ((power)/(1+(0.166666 * pow(eular, 0.00352 * (-left_x_d + 1300)))));}
    else {
      side_left_prof = -((power)/(1+(0.166666 * pow(eular, 0.00352  * (left_x_d + 1300)))));}

    if(right_xnd_d > 0) {
      side_right_prof = ((power)/(1+(0.166666 * pow(eular, 0.00352  * (-right_xnd_d + 1300)))));}
    else {
      side_right_prof = -((power)/(1+(0.166666 * pow(eular, 0.00352  * (right_xnd_d + 1300)))));}

    std::cout << side_left_prof << ":" << side_right_prof << ":" << left_x_d << ":" << right_xnd_d << "\n";
    
    leftFront.  spin(vex::directionType::fwd, side_left_prof, vex::percentUnits::pct);
    leftMiddle. spin(vex::directionType::fwd, side_left_prof, vex::percentUnits::pct);
    leftBack.   spin(vex::directionType::fwd, -side_left_prof, vex::percentUnits::pct);
    rightFront. spin(vex::directionType::fwd, -side_right_prof, vex::percentUnits::pct);
    rightMiddle.spin(vex::directionType::fwd, -side_right_prof, vex::percentUnits::pct);
    rightBack.  spin(vex::directionType::fwd, side_right_prof, vex::percentUnits::pct);

    vex::task::sleep(10);
  }

  leftFront.stop();
  leftMiddle.stop();
  leftBack.stop();

  rightFront.stop();
  leftMiddle.stop();
  rightBack.stop();

  //directional conditions
  pos.x_pos = pos.x_pos + (cos(radian)*(distancew));
  pos.y_pos = pos.y_pos + (sin(radian)*(distancew));

  this->robot_p->drive.obj->pos_devices.x_enc_nd->resetRotation();
  this->robot_p->drive.obj->pos_devices.x_enc->resetRotation();
  return;
}

#endif