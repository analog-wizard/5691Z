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

class robot_ {
  public:
    struct drive {
      uint8_t type;
      //In inches..
      double wh_size;
      //The most commmon design for tracking wheels requires the 2.75 inch Omni wheel, so 2.75 is the default
      double tr_wh_size = 2.75;
      //In (Driven wheel) RPM..
      double speed;
      //In inches..
      double x_size;
      double y_size;
    } drive;
    
    enum drive_type {
      tank_drive = 1,
      x_drive = 2,
      mecanum = 3,
      h_drive = 4,
      dragon_drive = 5,
      swerve = 6
    };
    
    //Constructor
    robot_(robot_::drive_type drive_, double wh_size, double tr_wh_size, double speed, double x_size, double y_size) {
      robot_::drive.type = drive_;
      robot_::drive.wh_size = wh_size;
      robot_::drive.tr_wh_size = tr_wh_size;
      robot_::drive.speed = speed;
      robot_::drive.x_size = x_size;
      robot_::drive.y_size = y_size;
    }
    robot_(robot_::drive_type drive_, double wh_size, double speed, double x_size, double y_size) {
      robot_::drive.type = drive_;
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
  protected:
    //Pointer to the robot class instance passed at field class construction
    robot_ *robot_p;

    //Positional data
    struct pos_ {
      int32_t x_pos;
      int32_t y_pos;
      //Z axis is for use in high hangs or other platform based mechanics
      double z_pos;
      double angle;
    } pos;

    //Eular's number, which is used in the Sigmoid curve motion profiling seen throughout the library
    const double eular = 2.718281828459045235;
  public:
    //Units used to determine in what increments the robot can move
    enum units {
      tile = 1000,
      foot = 100,
      inches = 10,
      raw = 1
    };
    //Constructor
    //*robot_1 is a "robot" class pointer used to refer to the physical aspects of the robot
    //*robot_p is always initialized in order to avoid a Segmentation fault, which can result from accessing a uninitialized pointer
    field(robot_ *robot_1, int32_t x_pos, int32_t y_pos, double z_pos, double angle) {
      //robot_p is assigned the value of robot_1 to allow access to the passed robot instance to all members of the "field" class
      robot_p = robot_1;
      pos.x_pos = x_pos; 
      pos.y_pos = y_pos; 
      pos.z_pos = z_pos;
      pos.angle = angle;
    };
    //Destructor
    ~field() {};
    
    //Used to print the position of the robot in an (x,y,z,angle) manner
    void display    (); 
    //Used to set the position of the robot in an (x,y,z,angle) manner
    void set_pos    (field::units units, int32_t x_pos, int32_t y_pos, double z_pos);

    //Used to rotate the robot to a desired angle at a desired power
    void rotate     (uint8_t power, double rotation); 
    //To rotate the robot in an arc to a given position at a given radius at a given power
    void arc        (uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, double radius, double distance_center);

    //To move in a straight line between the current position and a given point at a given speed
    //The "bool button" is used to enable barrier sensing, which would stop the robot should a obstacle was encountered
    void direct     (uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button);
    void direct     (uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, double angle, double angle_margin);
    void direct_time(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, uint16_t time);
};

class platform {
  protected:
    struct rel_pos {
      int32_t x_pos;
      int32_t y_pos;
      //Z axis is for use in high hangs or other platform based mechanics
      double z_pos;
      double angle;
    } rel_pos;
    int32_t x_limit;
    int32_t y_limit;
  public:
    //Constructor
    platform(int32_t x_pos, int32_t y_pos, double z_pos, double angle, int32_t x_limit, int32_t y_limit, field *field_obj) {
      rel_pos.x_pos = x_pos;
      rel_pos.y_pos = y_pos;
      rel_pos.z_pos = z_pos;
      rel_pos.angle = angle;
      platform::x_limit = x_limit;
      platform::y_limit = y_limit;
    };
    //Destructor
    ~platform() {};
    
    void display_();
    void tare();
};

void field::display() {
  std::cout << pos.x_pos << ": " << pos.y_pos << ": " << pos.z_pos << ": " << pos.angle << "\n";
}

void field::set_pos(field::units units, int32_t x_pos, int32_t y_pos, double z_pos) {
  field::pos.x_pos = x_pos;
  field::pos.y_pos = y_pos;
  field::pos.z_pos = z_pos;
}

void field::arc(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, double radius, double distance_center) {
  //distance_center : To represent distance between 

  //Measure with y_enc
  y_encoder.resetRotation();
  ynd_encoder.resetRotation();
  x_encoder.resetRotation();

  //910" PER MINUTE : 15 1/6" PER SECOND : 100% power

  //Switch/case statements cannot be used with double/float
    //Prefered as they do not recheck value of passed expression every case
  //Use atan2 to find quadrant of end point

  //0       : (0,inf)
  //1.5708  : (inf,0)
  //-1.5708 : (-inf,0)
  //3.14159 : (0,-inf)

  //All angles are in radians
  //X_value : Angle/1.5708
  //Y_value : Angle * (Angle/1.5708)

  //In inches
  double inner_cir = 2*M_PI*radius; //3.14
  //Add 13.5" (for the opposing side of the drive) (Left wheel axis to Right wheel axis)
  double outer_cir = 2*M_PI*(radius + 13.5); //87.96

  //Differential steer factor
  double dsf = (inner_cir/outer_cir);

  double D_pow = (dsf * power); //(For closest side of the drive to the center of the arc)
  double N_pow = (double)power;

  //                                                                  \/ robot::drive.wh_size
  //Add offset (If needed)
  double y_enc_conv = (ynd_encoder.rotation(vex::rotationUnits::deg) / robot_p->drive.wh_size) * 360;

  double radian = atan2(x_pos, y_pos);

  //                                                                                          \/ robot::drive.tr_wh_size
  //To rotate to angle appropriate for turning the correct arc
  double half_seg = ((sqrt(pow(x_pos - pos.x_pos, 2.0) + pow(y_pos - pos.y_pos, 2.0))/2)/360) * robot_p->drive.tr_wh_size;
  double small = radius - distance_center;
  double start_angle = 1.5708 - atan(small/half_seg);

  if(radian >= 0 && radian <= 3.1415) {
    double angle = pos.angle + ((radian/180)*M_PI);
    field::rotate(power, angle);
    while(y_enc_conv < outer_cir) {
      leftFront.spin(vex::directionType::fwd,  D_pow * ((200/(1+(pow(eular,-0.00652 * y_enc_conv)))-100)/100)/1000, vex::velocityUnits::pct);
      leftBack.spin(vex::directionType::fwd,   D_pow * ((200/(1+(pow(eular,-0.00652 * y_enc_conv)))-100)/100)/1000, vex::velocityUnits::pct);
      rightFront.spin(vex::directionType::fwd, N_pow * ((200/(1+(pow(eular,-0.00652 * y_enc_conv)))-100)/100)/1000, vex::velocityUnits::pct);
      rightBack.spin(vex::directionType::fwd,  N_pow * ((200/(1+(pow(eular,-0.00652 * y_enc_conv)))-100)/100)/1000, vex::velocityUnits::pct);
    };
  } else if(radian < 0 && radian >= -1.5708) {
    while(y_enc_conv < inner_cir) {
      //motion profile
    };
  } else if(radian < -1.5708 && radian <= -3.1415) {
    
  } else;

  ynd_encoder.resetRotation();
  y_encoder.resetRotation();
  x_encoder.resetRotation();

  leftFront.stop(vex::brakeType::coast);
  leftBack.stop(vex::brakeType::coast);
  rightFront.stop(vex::brakeType::coast);
  rightBack.stop(vex::brakeType::coast);
}

void field::rotate(uint8_t power, double rotation) {
  double angle_mes = imu.yaw();
  //const double eular = 2.718281828459045235;
  //double prof_power = ((200/(1+(pow(eular,-0.5 * rotation)))-100)/100) * angle_mes;

  leftFront.stop(vex::brakeType::hold);
  leftBack.stop(vex::brakeType::hold);
  rightFront.stop(vex::brakeType::hold);
  rightBack.stop(vex::brakeType::hold);

  if(rotation > 0) {
    while(angle_mes < rotation) {
      std::cout << imu.yaw() << "\n";
      angle_mes = imu.yaw();
      leftFront.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
      leftBack.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
      rightFront.spin(vex::directionType::fwd, -power, vex::velocityUnits::pct);
      rightBack.spin(vex::directionType::fwd, -power, vex::velocityUnits::pct);
      vex::task::sleep(10);
    }
  }
  else if(rotation > 0) {
    while(angle_mes < rotation) {
      angle_mes = imu.yaw();
      std::cout << imu.yaw() << "\n";
      leftFront.spin(vex::directionType::fwd, -power, vex::velocityUnits::pct);
      leftBack.spin(vex::directionType::fwd, -power, vex::velocityUnits::pct);
      rightFront.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
      rightBack.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
      vex::task::sleep(10);
    }
  }
  else;

  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();

  y_encoder.resetRotation();
  x_encoder.resetRotation();

  leftFront.stop(vex::brakeType::coast);
  leftBack.stop(vex::brakeType::coast);
  rightFront.stop(vex::brakeType::coast);
  rightBack.stop(vex::brakeType::coast);
}

void field::direct(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, double angle, double angle_margin) {
  double d_x_pos = (x_pos-pos.x_pos);
  double d_y_pos = (y_pos-pos.y_pos);
  
  double slope2;
  double slope1;

  double slope1_prof;
  double slope2_prof;

  double d_angle;

  long double radian = atan2(y_pos, x_pos);
  
  const double eular = 2.718281828459045235;

  long double distancew = (int)sqrt(pow((x_pos - pos.x_pos),2.00) + pow((y_pos - pos.y_pos),2.00));

  if(button) {
    while(fabs(distancew) > error_margin && !front_limit.pressing()) {
      pos.y_pos += (ynd_encoder.position(vex::rotationUnits::deg));
      pos.x_pos += (x_encoder.position(vex::rotationUnits::deg));
      
      ynd_encoder.resetRotation();
      x_encoder.resetRotation();

      d_x_pos = (x_pos-pos.x_pos);
      d_y_pos = (y_pos-pos.y_pos);

      radian = atan2(d_y_pos, d_x_pos);

      if(radian >= 0 && radian <= 1.570796) {
        slope1 = power * (-100 + ((radian/0.785398) * 100));
        slope2 = power * 100;
      }
      else if(radian > 1.570796 && radian <= 3.141592)   {
        slope1 = power * 100;
        slope2 = power * (100 - (((radian-1.570796)/0.785398) * 100));
      }
      else if(radian < 0 && radian >= -1.570796) {
        slope1 = power * -100;
        slope2 = power * (100 - ((fabs(radian)/0.785398) * 100));
      }
      else if(radian < -1.570796 && radian >= -3.141592) {
        slope1 = power * (-100 + ((fabs(radian)-1.570796)/0.785398) * 100);
        slope2 = power * -100;
      }
      else;

      d_angle = imu.heading() - angle;

      distancew = (int)sqrt(pow((x_pos - pos.x_pos),2.00) + pow((y_pos - pos.y_pos),2.00));
      slope1_prof = (slope1 * (200/(1+(pow(eular,-0.00652 * distancew)))-100)/100)/100;
      slope2_prof = (slope2 * (200/(1+(pow(eular,-0.00652* distancew)))-100)/100)/100;

      leftFront. spin(vex::directionType::fwd, ((slope1_prof/100) * 12000) - (40/(1+(pow(eular,-0.07 * distancew)))-20), vex::voltageUnits::mV);
      leftBack.  spin(vex::directionType::fwd, ((slope2_prof/100) * 12000) - (40/(1+(pow(eular,-0.07 * distancew)))-20), vex::voltageUnits::mV);
      rightFront.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000) - (40/(1+(pow(eular,-0.07 * distancew)))-20), vex::voltageUnits::mV);
      rightBack. spin(vex::directionType::fwd, ((slope1_prof/100) * 12000) - (40/(1+(pow(eular,-0.07 * distancew)))-20), vex::voltageUnits::mV);
      vex::task::sleep(10);
    }
  }
  else {
    while(fabs(distancew) > error_margin) {
      pos.y_pos += (ynd_encoder.position(vex::rotationUnits::deg));
      pos.x_pos += (x_encoder.position(vex::rotationUnits::deg));
      
      ynd_encoder.resetRotation();
      x_encoder.resetRotation();

      d_x_pos = (x_pos-pos.x_pos);
      d_y_pos = (y_pos-pos.y_pos);

      radian = atan2(d_y_pos, d_x_pos);

      if(radian >= 0 && radian <= 1.570796) {
        slope1 = power * (-100 + ((radian/0.785398) * 100));
        slope2 = power * 100;
      }
      else if(radian > 1.570796 && radian <= 3.141592)   {
        slope1 = power * 100;
        slope2 = power * (100 - (((radian-1.570796)/0.785398) * 100));
      }
      else if(radian < 0 && radian >= -1.570796) {
        slope1 = power * -100;
        slope2 = power * (100 - ((fabs(radian)/0.785398) * 100));
      }
      else if(radian < -1.570796 && radian >= -3.141592) {
        slope1 = power * (-100 + ((fabs(radian)-1.570796)/0.785398) * 100);
        slope2 = power * -100;
      }
      else;

      distancew = (int)sqrt(pow((x_pos - pos.x_pos),2.00) + pow((y_pos - pos.y_pos),2.00));
      slope1_prof = (slope1 * (200/(1+(pow(eular,-0.00652 * distancew)))-100)/100)/100;
      slope2_prof = (slope2 * (200/(1+(pow(eular,-0.00652 * distancew)))-100)/100)/100;

      leftFront.spin(vex::directionType::fwd, ((slope1_prof/100) * 12000), vex::voltageUnits::mV);
      leftBack.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000), vex::voltageUnits::mV);
      rightFront.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000), vex::voltageUnits::mV);
      rightBack.spin(vex::directionType::fwd, ((slope1_prof/100) * 12000), vex::voltageUnits::mV);
      vex::task::sleep(10);
    }
  }
  ynd_encoder.resetRotation();
  x_encoder.resetRotation();
  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();
  return;
}

void field::direct_time(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, uint16_t time) {
  leftFront.spin(vex::directionType::fwd, 1, vex::voltageUnits::mV);
  leftBack.spin(vex::directionType::fwd, 1, vex::voltageUnits::mV);
  rightFront.spin(vex::directionType::fwd, 1, vex::voltageUnits::mV);
  rightBack.spin(vex::directionType::fwd, 1, vex::voltageUnits::mV);
  vex::task::sleep(50);

  double d_x_pos = (x_pos-pos.x_pos);
  double d_y_pos = (y_pos-pos.y_pos);
  
  double slope2;
  double slope1;

  double slope1_prof;
  double slope2_prof;

  long double radian = atan2(d_y_pos, d_x_pos);
  
  const double eular = 2.718281828459045235;

  long double distancew = (int)sqrt(pow((x_pos - pos.x_pos),2.00) + pow((y_pos - pos.y_pos),2.00));

  vex::timer time_;
  if(button) {
    while(fabs(distancew) > error_margin && !front_limit.pressing() && time_.time() < time) {
      pos.y_pos += (ynd_encoder.position(vex::rotationUnits::deg));
      pos.x_pos += (x_encoder.position(vex::rotationUnits::deg));
      
      ynd_encoder.resetRotation();
      x_encoder.resetRotation();

      d_x_pos = (x_pos-pos.x_pos);
      d_y_pos = (y_pos-pos.y_pos);

      radian = atan2(d_y_pos, d_x_pos);

      if(radian >= 0 && radian <= 1.570796) {
        slope1 = power * (-100 + ((radian/0.785398) * 100));
        slope2 = power * 100;
      }
      else if(radian > 1.570796 && radian <= 3.141592)   {
        slope1 = power * 100;
        slope2 = power * (100 - (((radian-1.570796)/0.785398) * 100));
      }
      else if(radian < 0 && radian >= -1.570796) {
        slope1 = power * -100;
        slope2 = power * (100 - ((fabs(radian)/0.785398) * 100));
      }
      else if(radian < -1.570796 && radian >= -3.141592) {
        slope1 = power * (-100 + ((fabs(radian)-1.570796)/0.785398) * 100);
        slope2 = power * -100;
      }
      else;

      distancew = (int)sqrt(pow((x_pos - pos.x_pos),2.00) + pow((y_pos - pos.y_pos),2.00));
      slope1_prof = (slope1 * ((power*2)/(1+(pow(eular,-0.00652 * distancew)))-power)/100)/100;
      slope2_prof = (slope2 * ((power*2)/(1+(pow(eular,-0.00652 * distancew)))-power)/100)/100;

      leftFront.spin(vex::directionType::fwd, ((slope1_prof/100) * 12000), vex::voltageUnits::mV);
      leftBack.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000), vex::voltageUnits::mV);
      rightFront.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000), vex::voltageUnits::mV);
      rightBack.spin(vex::directionType::fwd, ((slope1_prof/100) * 12000), vex::voltageUnits::mV);
      vex::task::sleep(10);
    }
  }
  else {
    while(fabs(distancew) > error_margin && time_.time() < time) {
      pos.y_pos += (ynd_encoder.position(vex::rotationUnits::deg));
      pos.x_pos += (x_encoder.position(vex::rotationUnits::deg));
      
      ynd_encoder.resetRotation();
      x_encoder.resetRotation();

      d_x_pos = (x_pos-pos.x_pos);
      d_y_pos = (y_pos-pos.y_pos);

      radian = atan2(d_y_pos, d_x_pos);

      if(radian >= 0 && radian <= 1.570796) {
        slope1 = power * (-100 + ((radian/0.785398) * 100));
        slope2 = power * 100;
      }
      else if(radian > 1.5707963 && radian <= 3.141592)   {
        slope1 = power * 100;
        slope2 = power * (100 - (((radian-1.5707963)/0.785398) * 100));
      }
      else if(radian < 0 && radian >= -1.570796) {
        slope1 = power * -100;
        slope2 = power * (100 - ((fabs(radian)/0.785398) * 100));
      }
      else if(radian < -1.570796 && radian >= -3.141592) {
        slope1 = power * (-100 + ((fabs(radian)-1.570796)/0.785398) * 100);
        slope2 = power * -100;
      }
      else;

      distancew = (int)sqrt(pow((x_pos - pos.x_pos),2.00) + pow((y_pos - pos.y_pos),2.00));
      slope1_prof = (slope1 * ((power*2)/(1+(pow(eular,-0.00652 * distancew)))-power)/100)/100;
      slope2_prof = (slope2 * ((power*2)/(1+(pow(eular,-0.00652 * distancew)))-power)/100)/100;

      leftFront.spin(vex::directionType::fwd, ((slope1_prof/100) * 12000), vex::voltageUnits::mV);
      leftBack.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000), vex::voltageUnits::mV);
      rightFront.spin(vex::directionType::fwd, ((slope2_prof/100) * 12000), vex::voltageUnits::mV);
      rightBack.spin(vex::directionType::fwd, ((slope1_prof/100) * 12000), vex::voltageUnits::mV);
      vex::task::sleep(10);
    }
  }
  ynd_encoder.resetRotation();
  x_encoder.resetRotation();
  leftFront.stop();
  leftBack.stop();
  rightFront.stop();
  rightBack.stop();
  return;
}

#endif