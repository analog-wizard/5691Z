/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ian                                                       */
/*    Created:      Sat Nov 07 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "init.h"
#include "auton.h"

using namespace vex;

void pre_auton ( void ) {
  //Unknown the effect of calibrating from the V5 Device Menu over from within the program: Requires Testing
  //Calibrating from within the program stops the IMU from updating for 3 seconds (the recommended time)
  //Calibrating from the V5 Device Menu
  
  /*imu.calibrate();
  vex::task::sleep(3000);
  while(imu.isCalibrating()) {} //Waits until the IMU has finished calibrating
  std::cout << "Imu Finished" << "\n"; //Prints when the IMU has finished calibrating */
  //Above is a system that informs the user of the given program when the IMU has finished calibrating

  /*for(int8_t i = 0; i++; i<=8) {
    controller_.Screen.setCursor(3, i);
    if (vex::motor((port_array[i] - 1)).installed())
      controller_.Screen.print("1");
    else
      controller_.Screen.print("0");
  }*/ 
  //Above is a system designed to print onto the Controller LCD screen a string of symbols,
  // these symbols each represent a motor, with a "1" representing a motor installed within the inspected port
  // and "0" representing a motor is missing from the inspected port
  //These numbers are printed in order they are provided in an array named "port_array"
  //Unfortunately, this system does not currently print to the Controller LCD for unknown reasons
  
  
  //Reset all encoder values in order to prevent error from left over values from adjusting the robot
  leftFront.resetRotation();
  leftBack.resetRotation();
  rightFront.resetRotation();
  rightBack.resetRotation();
  rightIntake.resetRotation();
  leftIntake.resetRotation();
  y_encoder.resetRotation();
  x_encoder.resetRotation();
}

void driveControl() {
  if(controller_.ButtonX.pressing() /*&& not passing the lower limit*/) {
      //Combination of drive motors to result in the lowering of the mogo lift
  } else if(controller_.ButtonB.pressing() /*&& not passing the upper limit*/) {
      //Combination of drive motors to result in the raising of the mogo lift
  } else {
  //Fix (if needed) the output of the motors during normal drive functions as to avoid activating the mogo

  //controller_.Axis2.value() returns a integer value of -127 to 127 based upon the position of the left joystick along Axis3,
  // while the maximum power of the motors is -100 to 100, this provides a "deadspace", or a margin in which the joystick
  // can be positioned within and still achieve maximum power.
  leftFront.spin(vex::directionType::fwd,  controller_.Axis2.value(), vex::velocityUnits::pct);
  leftBack.spin(vex::directionType::fwd,   controller_.Axis2.value(), vex::velocityUnits::pct);
  //Use Axis3 of the right joystick to controller the right side of the tank drive
  rightBack.spin(vex::directionType::fwd,  controller_.Axis3.value(), vex::velocityUnits::pct);
  rightFront.spin(vex::directionType::fwd, controller_.Axis3.value(), vex::velocityUnits::pct);
  }
}

//This function's purpose is to display positional/angular information from the robot via the inbuilt IDE Terminal
//This is mainly used for Autonomous (often called the "Auton" or "Auto") period, allowing quick construction of a new Autonomous program
void autonMeasure() {

}

void vibrate_time(vex::timer *timer__) {
  //The provided timer object will return time in milliseconds (chosen over seconds due to precision) counting up from 0
  // The maximum time, within a 1:45 minute match is 105 seconds, after this time has been passed, this function will do nothing.
  // The purpose of this function is to serve as a backup for the Drive Team members, as one of their (assigned) jobs is to call out time
  // for the driver, this program will signal the remote to vibrate at these pre determined times incase the Drive team members are needed
  // to "call out" or relay opposing alliance information or a change in game strategy to the driver
  
  if((int)timer__->time() > 105000) { //If the provided timer object is greater than 105 seconds (1:45) then do nothing
  } else if(((int)timer__->time() % 10000) == 0 && !(timer__->time() == 75000) && !(timer__->time() >= 90000)) {
    //If the value of the provided timer object is not equal to 75 seconds and not less or equal to than 90 seconds (90000 milliseconds) 
    // and has a dividend of 0 (when divided by 10 seconds or 10000 milliseconds) then the controller will vibrate 1 long pulse and 1 short pulse
    controller_.rumble("-.");
  } else if((timer__->time() == 75000)) {
    //If the value of the provided timer object is equal to 75 seconds (75000 milliseconds) (30 seconds left within a 1:45 match), then the controller
    // will vibrate 2 short pulses, 1 long pulse, and 1 short pulse
    controller_.rumble("..-.");
  } else if(((int)timer__->time() % 1000) == 0 && (timer__->time() > 75000) && !(timer__->time() < 90000)) {
    //If the value of the provided timer object is not less than 90 seconds (15 seconds left in a 1:45 match) and is greater than 75 seconds
    // and has a dividend of 0 (when divided by 1 second or 1000 milliseconds) 
    controller_.rumble(".");
  } else;
}

//The function which contains all functions utilized by the Driver Control period
void usercontrol ( void ) {
  vex::timer _timer; //Create a timer object for use in the "vibrate_time" function
  // This timer object is defined outside of the scope of "vibrate_time" as to allow
  // use by other functions should need be.
  
  //Set all motor brakeTypes to coast before Driver Control to prevent confusion or inconvience
  leftFront.stop(vex::brakeType::coast);
  leftBack.stop(vex::brakeType::coast);
  rightFront.stop(vex::brakeType::coast);
  rightBack.stop(vex::brakeType::coast);  
  
  while(true) {
    vex::thread tdriveControl(driveControl); //Creates a Vex thread of function "driveControl" named "tdriveControl"
    vex::thread tautonMeasure(autonMeasure); //Creates a Vex thread of function "autonMeasure" named "tautonMeasure"
    vibrate_time(&_timer); //Pass by reference the "_timer" timer object
    // Passing by reference keeps the function much less dependent on objects/members/varibles it is given, as it does not see the name
    // but only the address, allowing name changes and edits to the _timer object to be much easier. Passing by reference also negates the
    // memory usage of passing by value (E.G. vibrate_time(_timer);) which would create a copy of _timer, whereas passing by reference does not
    vex::task::sleep(10); //Wait .01 of a second to allow other threads to run
  }
}

int main() {
  competition_.autonomous   (autonomous); //Activates the function "autonomous" during the Autonomous Period
  competition_.drivercontrol(usercontrol);//Activates the function "usercontrol" during the Driver Control period

  pre_auton(); //The "pre_auton" function will run at the start of the program, after which the program will fall into
  // a while loop (a disabled mode) which waits every .1 of a second, after which it will check if the robot has exited the disabled mode

  while(1) {
    vex::task::sleep(100);
  }
}
