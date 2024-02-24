/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       whiteeth                                                  */
/*    Created:      2/22/2024, 8:25:04 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

motor topLeftFront = motor(PORT8, ratio6_1, true);
motor bottomLeftFront = motor(PORT9, ratio6_1, false);
motor bottomLeftBack = motor(PORT7, ratio6_1, false);

motor_group leftMotors = motor_group(topLeftFront, bottomLeftFront, bottomLeftBack);

motor topRightFront = motor(PORT3, ratio6_1, true);
motor bottomRightFront = motor(PORT4, ratio6_1, false);
motor bottomRightBack = motor(PORT5, ratio6_1, false);

motor_group rightMotors = motor_group(topRightFront, bottomRightFront, bottomRightBack);

controller Controller1 = controller(primary);

motor arm = motor(PORT6, ratio18_1, false);
motor intake = motor(PORT10, ratio18_1, false);

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    float moveSpeed = 1;
    float spinSpeed = .5;
    
    leftMotors.spin(forward,
               (-Controller1.Axis3.position()*moveSpeed - Controller1.Axis1.position()*spinSpeed),
               pct);

    rightMotors.spin(forward,
                (Controller1.Axis3.position()*moveSpeed - Controller1.Axis1.position()*spinSpeed),
                pct);

    if (Controller1.ButtonR1.pressing())
    {
      intake.spin(reverse, 100, percent);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      intake.spin(forward, 100, percent);
    }
    else
    {
      intake.stop(brake);
    }
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
