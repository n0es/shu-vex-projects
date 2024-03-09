/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       whiteeth                                                  */
/*    Created:      2/22/2024, 7:03:14 PM                                     */
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

motor arm = motor(PORT16, ratio18_1, false);

motor liftLeft = motor(PORT12, ratio18_1, false);
motor liftRight = motor(PORT14, ratio18_1, true);

motor_group lift = motor_group(liftLeft, liftRight);

controller Controller1 = controller(primary);

inertial imu = inertial(PORT17);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void move(float inches, vex::directionType direction, float velocity = NULL)
{
  leftMotors.resetPosition();
  rightMotors.resetPosition();
  double revs = (inches / 6.28) * (1 - (2 * int(direction)));
  // (1 - (2 * int(direction))) is 1 when direction==FWD and -1 when direction==REV
  if (velocity)
  {
    leftMotors.spinFor(-revs, rev, velocity, vex::velocityUnits::pct, false);
    rightMotors.spinFor(revs, rev, velocity, vex::velocityUnits::pct, true);
  }
  else
  {
    leftMotors.spinFor(-revs, rev, false);
    rightMotors.spinFor(revs, rev, true);
  }

  leftMotors.stop(hold);
  rightMotors.stop(hold);
}

enum rotationDirection
{
  CCW = 0,
  CW,
  CounterClockwise = 0,
  Clockwise
};

void rotate(float degrees, rotationDirection direction, float velocity = NULL)
{
  leftMotors.resetPosition();
  rightMotors.resetPosition();

  float raws = (7.5 * degrees) * (1 - (2 * int(direction)));
  if (velocity)
  {
    leftMotors.spinFor(raws, rotationUnits::raw, velocity, vex::velocityUnits::pct, false);
    rightMotors.spinFor(raws, rotationUnits::raw, velocity, vex::velocityUnits::pct, true);
  }
  else
  {
    leftMotors.spinFor(raws, rotationUnits::raw, false);
    rightMotors.spinFor(raws, rotationUnits::raw, true);
  }

  leftMotors.stop(hold);
  rightMotors.stop(hold);
}

void ReCalibrateGyro()
{
  imu.calibrate();
  while( imu.isCalibrating() )
  { wait(10,msec); }
}

void pre_auton(void)
{
  // PRE AUTON SUCKS!!!!
  ReCalibrateGyro();
}


// used for testing motor revolutions for one rotation

// void autonomous(void)
// {
//   ReCalibrateGyro();
//   leftMotors.spin(forward, 10, percent);
//   rightMotors.spin(forward, 10, percent);
//   while (abs(imu.rotation(rotationUnits::deg)) < 360)
//   {
//     vex::wait(50, msec);
//     printf("%f \n", imu.rotation(deg));
//   }
//   leftMotors.stop(hold);
//   rightMotors.stop(hold);
//   printf("360: %f \n", left1.position(rotationUnits::raw));
//   float rpd = left1.position(rotationUnits::raw) / 360;
//   printf("rpd: %f \n", rpd);
// }

void autonomous(void)
{
  move(21, forward, 20);
  rotate(45, CCW, 20);
  move(15, reverse, 20);

  rotate(10, CW, 30);
  arm.spinTo(340, degrees, true);

  int repeat = 22;
  while (repeat > 0)
  {
    repeat--;
    rotate(27, CCW, 30);
    rotate(25, CW, 30);
    vex::wait(1, sec);
  }
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
      arm.stop(brakeType::hold);
    }

    if (Controller1.ButtonL1.pressing())
    {
      lift.spin(directionType::fwd, 100.0, percentUnits::pct);
    }
    else if (Controller1.ButtonL2.pressing())
    {
      lift.spin(directionType::rev, 100.0, percentUnits::pct);
    }
    else
    {
      lift.stop(brakeType::hold);
    }

    leftMotors.spin(forward,
                    (-Controller1.Axis3.position() - Controller1.Axis1.position()),
                    percent);

    rightMotors.spin(forward,
                     (Controller1.Axis3.position() - Controller1.Axis1.position()),
                     percent);

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
