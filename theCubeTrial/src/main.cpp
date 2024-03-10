/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       whiteeth                                                  */
/*    Created:      2/29/2024, 8:43:05 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "math.h"

using namespace vex;

// A global instance of competition
competition Competition;

motor topLeftFront = motor(PORT8, ratio6_1, true);
motor topLeftBack = motor(PORT20, ratio6_1, true);
motor bottomLeftFront = motor(PORT9, ratio6_1, false);
motor bottomLeftBack = motor(PORT7, ratio6_1, false);

motor_group leftMotors = motor_group(topLeftFront, topLeftBack, bottomLeftFront, bottomLeftBack);

motor topRightFront = motor(PORT3, ratio6_1, true);
motor topRightBack = motor(PORT11, ratio6_1, true);
motor bottomRightFront = motor(PORT4, ratio6_1, false);
motor bottomRightBack = motor(PORT5, ratio6_1, false);

motor_group rightMotors = motor_group(topRightFront, topRightBack, bottomRightFront, bottomRightBack);

motor blockerLeft = motor(PORT19, ratio36_1, true);
motor blockerRight = motor(PORT12, ratio36_1, false);

motor_group blocker = motor_group(blockerLeft, blockerRight);

controller Controller1 = controller(primary);

motor arm = motor(PORT6, ratio18_1, false);
motor intake = motor(PORT10, ratio18_1, false);

triport ThreeWirePort(PORT22);
pneumatics wings = pneumatics(ThreeWirePort.H);

inertial imu = inertial(PORT11);

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

  float revs = ((((10.35 * 3.14) / 360) * degrees) / 6.28) * (1 - (2 * int(direction)));
  if (velocity)
  {
    leftMotors.spinFor(revs, rev, velocity, vex::velocityUnits::pct, false);
    rightMotors.spinFor(revs, rev, velocity, vex::velocityUnits::pct, true);
  }
  else
  {
    leftMotors.spinFor(revs, rev, false);
    rightMotors.spinFor(revs, rev, true);
  }

  leftMotors.stop(brake);
  rightMotors.stop(brake);
}

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
  move(27, reverse, 20);
  rotate(45, CW, 20);
  move(19.5, forward, 20);

  rotate(75, CCW, 30);
  blocker.spinTo(470,degrees, true);
  arm.spinTo(176, degrees, true);
  
  blocker.spinTo(13,degrees, true);
  int repeat = 22;
  while (repeat > 0)
  {
    repeat--;
    if (repeat==11)
    {
      move(2,reverse,20);
    }
    rotate(30, CW, 60);
    vex::wait(1,msec);
    rotate(20, CCW, 35);
    vex::wait(1, sec);
  }
  blocker.spinTo(470,degrees,true);
  arm.spinTo(1, degrees, true);
  blocker.spinTo(13,degrees, true);
  move(15.5,reverse,20);
  rotate(35,CW,20);
  move(78,reverse,60);
  rotate(45,CW,20);

  move(15,reverse,60);
  rotate(45,CW,20);
  move(15,reverse,60);
  move(10,forward,60);
  move(10,reverse,80);


  
  



 


  // arm.spinTo(0,degrees,true);
  // move(1,forward,20);
  // rotate(85,CW,30);
  // move(18,forward,20);
  // rotate(20, CCW, 30);
  // move(3, forward, 20);
  // rotate(26, CCW, 25);
  // move(2.75, forward, 15);
  // rotate(4, CCW, 25);
  // move(25,forward,20);
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
  float moveSpeed = 1;
  float spinSpeed = .65;
  while (1)
  {

    if (Controller1.ButtonL1.pressing())
    {
      if (Controller1.ButtonDown.pressing())
      {
        rightMotors.spin(reverse, 40, percent);
        leftMotors.spin(reverse, 40, percent);
      }
      else
      {
        rightMotors.stop(brake);
        leftMotors.stop(brake);
      }
    }
    else
    {
      leftMotors.spin(forward,
                      (-Controller1.Axis3.position() * moveSpeed - Controller1.Axis1.position() * spinSpeed),
                      pct);

      rightMotors.spin(forward,
                       (Controller1.Axis3.position() * moveSpeed - Controller1.Axis1.position() * spinSpeed),
                       pct);
    }

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