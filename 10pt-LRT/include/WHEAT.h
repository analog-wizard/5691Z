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

//A class which carries all relevant information used for positional tracking for Subsystem 1
class robot_ {
  public: //Able to be accessed by any function or class
    struct drive {
      //The type of drive represented by a integer ranging from 0-255,
      // these types and their number conversions are represented in the enumeration "drive_type"
      uint8_t type;
      
      //In inches..
      double wh_size; //Driven wheel size, as to determine speed and distance
      double x_size;  //Size X of the robot drive, as to be used for platforms
      double y_size;  //Size Y of the robot drive, as to be used for platforms
      //Both X and Y are used in tandum to determine the size of the robot as to not drive off of a platform
      
      //Tracking wheel size, used to determine measured speed and distance
      //The most commmon design for tracking wheels uses the 2.75 inch Omni wheel, so 2.75 is the default
      double tr_wh_size = 2.75;
      
      //In (Driven wheel) RPM, used to determine speed
      double speed;
    } drive;
    
    enum drive_type {
      tank_drive = 1,
      x_drive = 2,
      mecanum = 3,
      h_drive = 4,
      dragon_drive = 5,
      swerve = 6
    };

    //Constructor: Passes given variables to the class members
    robot_(robot_::drive_type drive_, double wh_size, double tr_wh_size, double speed, double x_size, double y_size) {
      robot_::drive.type = drive_; //Passes the drive type given at the "construction" (or initialization) to the class member "drive.type"
      robot_::drive.wh_size = wh_size;
      robot_::drive.tr_wh_size = tr_wh_size;
      robot_::drive.speed = speed;
      robot_::drive.x_size = x_size;
      robot_::drive.y_size = y_size;
    }//Example: robot_ robot(robot_::drive_type::tank_drive, 4.0, 4.0, 200, X, Y);
     //This creates a "robot_" class instance with the name "robot" with 4 inch 200 RPM driven wheels, 4 inch tracking wheels, an X dimension of X and a Y dimension of Y
    robot_(robot_::drive_type drive_, double wh_size, double speed, double x_size, double y_size) {
      robot_::drive.type = drive_;
      robot_::drive.wh_size = wh_size;
      robot_::drive.speed = speed;
      robot_::drive.x_size = x_size;
      robot_::drive.y_size = y_size;
    }//Example: robot_ robot(robot_::drive_type::tank_drive, 4.0, 200, X, Y)
    //This creates a "robot_" class instance with the name "robot" with 4 inch 200 RPM driven wheels, 2.75 inch tracking wheels, an X dimension of X and a Y dimension of Y
  
    //Destructor: Destroys the class instance invoked
    ~robot_() {}; //Example: "~robot();" would deconstruct a "robot_" class instance of "robot"
};

class field {
  friend class platform; //Allows instances of the "platform" class to access both protected and private members and fucntions
  protected: //Avalible to friend classes and child classes
    //Pointer to a robot class instance used for various calculations
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
  public: //Able to be accessed by any function or class
    //Units of movement
    enum units {
      tile = 1000,
      foot = 100,
      inches = 10,
      raw = 1     //Raw encoder Units
    };
    //Constructor
    //*robot_1 is a "robot" class pointer used to refer to an instance of the class "robot_"
    //*robot_p is always initialized in order to avoid a Segmentation fault, which can result from accessing a uninitialized pointer
    field(robot_ *robot_1, int32_t x_pos, int32_t y_pos, double z_pos, double angle) {
      //robot_p is assigned the value of robot_1 to allow access to the passed "robot_" instance to all functions and members of the "field" class
      robot_p = robot_1;
      pos.x_pos = x_pos;
      pos.y_pos = y_pos;
      pos.z_pos = z_pos;
      pos.angle = angle;
    };
    //Destructor
    ~field() {};
    
    //Used to print the position of the robot in an (x,y,z,angle) format
    void display    (); 
    //Used to set the position of the robot in an (x,y,z,angle) format (Testing Only)
    void set_pos    (field::units units, int32_t x_pos, int32_t y_pos, double z_pos);

    //Used to rotate the robot to a desired angle at a desired power/speed
    void rotate     (uint8_t power, double rotation, bool relative, double margin); 
  
    //WIP
    //To rotate the robot in an arc to a given position at a given radius at a given power
    void arc        (uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, double radius, double distance_center);

    //To move in a straight line between the current position and a given point at a given speed
    //The "bool button", when true, is used to enable obsticle sensing, which would stop the robot should a obstacle be encountered
    void direct     (uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin);
    void direct_time(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, uint16_t time);
};

class platform {
  protected:
    struct rel_pos {
      int32_t x_pos;
      int32_t y_pos;
      
      //the Z axis is used for high hangs or other platform based mechanics
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
    void set_pos();
};

//A function that prints out all postitional data in a (X,Y,Z,angle) format
void field::display() {
  std::cout << pos.x_pos << ": " << pos.y_pos << ": " << pos.z_pos << ": " << pos.angle << "\n";
}

//"set_pos" is for no other use other than testing movements or other features
void field::set_pos(field::units units, int32_t x_pos, int32_t y_pos, double z_pos) {
  field::pos.x_pos = x_pos;
  field::pos.y_pos = y_pos;
  field::pos.z_pos = z_pos;
}

//A "half-successful" attempt at making the robot (2020-2021) move in an arc, while the robot did move in an arc, the arc was not the specified
// size, relegating this function to "half-successful", further study is needed before reconstruction or repair of this fucntion
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

  //Think Square:
  //162.564853453/247.3878551
  //dsf = 0.65712544129

  //In inches
  double inner_cir = 2*M_PI*radius;
  //Add 13.5" (for the opposing side of the drive) (Left wheel axis to Right wheel axis)
  double outer_cir = 2*M_PI*(radius + 13.5);

  //Differential steer factor
  double dsf = (inner_cir/outer_cir);

  double D_pow = (dsf * power); //(For closest side of the drive to the center of the arc)
  double N_pow = (double)power;

  //                                                                  \/ robot::drive.wh_size
  //Add offset (If needed)
  double y_enc_conv = (ynd_encoder.rotation(vex::rotationUnits::deg) / 360) * robot_p->drive.tr_wh_size;

  double radian = atan2(x_pos, y_pos);

  //                                                                                          \/ robot::drive.tr_wh_size
  //To rotate to angle appropriate for turning the correct arc
  double half_seg = ((sqrt(pow(x_pos - pos.x_pos, 2.0) + pow(y_pos - pos.y_pos, 2.0))/360) * robot_p->drive.wh_size)/2;
  double small = radius - distance_center;
  double start_angle = (1.5708 - atan(small/half_seg)) * (180/M_PI);

  if(radian >= 0 && radian <= 3.1415) {
    double angle = pos.angle + ((radian/180)*M_PI);
    field::rotate(25, start_angle, false, 1);
    std::cout << D_pow * ((200/(1+(pow(eular,-0.00652 * (outer_cir - y_enc_conv))))-100)/100) << "\n";
    std::cout << outer_cir << ":" << y_enc_conv << ":" << (outer_cir - y_enc_conv) << "\n";
    while(outer_cir > y_enc_conv) {
      y_enc_conv = (ynd_encoder.rotation(vex::rotationUnits::deg) / 360) * robot_p->drive.tr_wh_size;
      std::cout << y_enc_conv << ":" << outer_cir << ":" << ynd_encoder.rotation(vex::rotationUnits::deg) << "\n";
      leftFront.spin(vex::directionType::fwd,  D_pow /* * ((200/(1+(pow(eular,-0.00652 * (outer_cir - y_enc_conv))))-100)/100)*/, vex::velocityUnits::pct);
      leftBack.spin(vex::directionType::fwd,   D_pow /* * ((200/(1+(pow(eular,-0.00652 * (outer_cir - y_enc_conv))))-100)/100)*/, vex::velocityUnits::pct);
      rightFront.spin(vex::directionType::fwd, N_pow /* * ((200/(1+(pow(eular,-0.00652 * (outer_cir - y_enc_conv))))-100)/100)*/, vex::velocityUnits::pct);
      rightBack.spin(vex::directionType::fwd,  N_pow /* * ((200/(1+(pow(eular,-0.00652 * (outer_cir - y_enc_conv))))-100)/100)*/, vex::velocityUnits::pct);
      vex::task::sleep(10);
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

//Relative and non-Relative movements provide abundant opportunity for confusion, as such, all angle based movements will be absolute
void field::rotate(uint8_t power, double rotation, double margin) {
  double angle_mes = imu.heading();
  double angle_mes_2 = 0;

  leftFront.stop(vex::brakeType::hold);
  leftBack.stop(vex::brakeType::hold);
  rightFront.stop(vex::brakeType::hold);
  rightBack.stop(vex::brakeType::hold);

  if(rotation < 0) {
    if(relative) {
      double position = 0;
      while(position < rotation) {
        std::cout << position << "\n";
        if(angle_mes_2 < 10 && angle_mes_2 > -10) {
          position += angle_mes_2;
        } else;

        leftFront.spin(vex::directionType::fwd,  power, vex::velocityUnits::pct);
        leftBack.spin(vex::directionType::fwd,   power, vex::velocityUnits::pct);
        rightFront.spin(vex::directionType::fwd, -power, vex::velocityUnits::pct);
        rightBack.spin(vex::directionType::fwd,  -power, vex::velocityUnits::pct);

        angle_mes = imu.heading();
        vex::task::sleep(20);
        angle_mes_2 = angle_mes - imu.heading();
      }
    } else {
      while(field::pos.angle < rotation) {
        std::cout << field::pos.angle << "\n";
        if(angle_mes_2 < 10 && angle_mes_2 > -10) {
          field::pos.angle += angle_mes_2;
        } else;

        leftFront.spin(vex::directionType::fwd,  power, vex::velocityUnits::pct);
        leftBack.spin(vex::directionType::fwd,   power, vex::velocityUnits::pct);
        rightFront.spin(vex::directionType::fwd, -power, vex::velocityUnits::pct);
        rightBack.spin(vex::directionType::fwd,  -power, vex::velocityUnits::pct);

        angle_mes = imu.heading();
        vex::task::sleep(20);
        angle_mes_2 = angle_mes - imu.heading();
      }
    }
  } else if(rotation > 0) {
    if(relative) {
      double position = 0;
      while(position < rotation) {
        std::cout << position << "\n";
        if(angle_mes_2 < 10 && angle_mes_2 > -10) {
          position += angle_mes_2;
        } else;

        leftFront.spin(vex::directionType::fwd,  -power, vex::velocityUnits::pct);
        leftBack.spin(vex::directionType::fwd,   -power, vex::velocityUnits::pct);
        rightFront.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
        rightBack.spin(vex::directionType::fwd,  power, vex::velocityUnits::pct);

        angle_mes = imu.heading();
        vex::task::sleep(20);
        angle_mes_2 = angle_mes - imu.heading();
      }
      std::cout << "Rotation of ~" << rotation << " degrees completed : True measure: " << position << "\n";
    } else {
      while(field::pos.angle < rotation) {
        std::cout << field::pos.angle << "\n";
        if(angle_mes_2 < 10 && angle_mes_2 > -10) {
          field::pos.angle += angle_mes_2;
        } else;

        leftFront.spin(vex::directionType::fwd,  -power, vex::velocityUnits::pct);
        leftBack.spin(vex::directionType::fwd,   -power, vex::velocityUnits::pct);
        rightFront.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
        rightBack.spin(vex::directionType::fwd,  power, vex::velocityUnits::pct);

        angle_mes = imu.heading();
        vex::task::sleep(20);
        angle_mes_2 = angle_mes - imu.heading();
      }
      std::cout << "Rotation to ~" << rotation << " (degrees) completed : True measure: " << angle_mes_2 << "\n";
    }
  } else;

  leftFront.stop(vex::brakeType::brake);
  leftBack.stop(vex::brakeType::brake);
  rightFront.stop(vex::brakeType::brake);
  rightBack.stop(vex::brakeType::brake);

  y_encoder.resetRotation();
  x_encoder.resetRotation();
}

void field::direct(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button) {
//Under Re-Construction: Several aspects of this fucntion were over-engineered (in that features could be created with much simpler means)
  //To Add: Drive dependant movement options
          //Simpler obsticle detection
          //Condensed pathing (or manuvering to the desired target)
  //Angle To Point
  double d_xpos = x_pos - pos.xpos;
  double d_ypos = y_pos - pos.ypos;
  double ATP = atan2(d_ypos, d_xpos) - ((pos.angle * M_PI)/180);
  double distance = sqrt(pow(pos.xpos-x_pos,2.0) + pow(pos.ypos-y_pos,2.0));
  field::rotate(power, ATP, error_margin);
  if(robot_p->drive.type == robot_::drive_type::tank_drive) {
    while(fabs(distance) > error_margin) {
      distance = sqrt(pow(pos.xpos-x_pos,2.0) + pow(pos.ypos-y_pos,2.0));
      
      //if(button && button_physical) {return;} //If the button being pressed and the button is enabled, then exit the function.
    }
  } else; //"else" planned for later
}

void field::direct_time(uint8_t power, field::units units, int32_t x_pos, int32_t y_pos, double error_margin, bool button, uint16_t time) {
//Under Re-Construction: Several aspects of this fucntion were over-engineered (in that features could be created with much simpler means)
  //To Add: Drive dependant movement options
          //Simpler obsticle detection
          //Condensed pathing (or manuvering to the desired target)
          //Simpler timer usage
}

#endif
