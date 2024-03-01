/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       whiteeth                                                  */
/*    Created:      2/22/2024, 8:04:09 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

motor left1 = motor(PORT19, ratio6_1, false);
motor left2 = motor(PORT20, ratio6_1, false);
motor left3 = motor(PORT10, ratio6_1, false);
motor left4 = motor(PORT9, ratio6_1, false);

motor_group leftMotors = motor_group(left1, left2, left3, left4);

motor right1 = motor(PORT13, ratio6_1, false);
motor right2 = motor(PORT11, ratio6_1, false);
motor right3 = motor(PORT1, ratio6_1, false);
motor right4 = motor(PORT2, ratio6_1, false);

motor_group rightMotors = motor_group(right1, right2, right3, right4);

controller Controller1 = controller(primary);

motor arm = motor(PORT16, ratio18_1, false);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

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

void autonomous(void)
{
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  arm.spinTo(.5, rotationUnits::rev);
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

void usercontrol(void)
{
  // User control code here, inside the loop
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    if (Controller1.ButtonR1.pressing())
    {
      arm.spin(directionType::fwd, 50.0, percentUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      arm.spin(directionType::rev, 50.0, percentUnits::pct);
    }
    else
    {
      arm.stop(brakeType::brake);
    }

    float moveSpeed = 1;
    float spinSpeed = .7;

    leftMotors.spin(forward,
                    (-Controller1.Axis3.position() * moveSpeed - Controller1.Axis1.position() * spinSpeed),
                    pct);

    rightMotors.spin(forward,
                     (Controller1.Axis3.position() * moveSpeed - Controller1.Axis1.position() * spinSpeed),
                     pct);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
