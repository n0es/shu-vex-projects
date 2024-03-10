/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       whiteeth                                                  */
/*    Created:      2/29/2024, 8:43:05 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

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

inertial imu = inertial(PORT15);

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

// used for testing motor revolutions for one rotation
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

  float raws = (4.33 * degrees) * (1 - (2 * int(direction)));
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
  while (imu.isCalibrating())
  {
    wait(10, msec);
  }
}

// void autonomous(void)
// {
//   ReCalibrateGyro();
//   leftMotors.spin(forward, 5, percent);
//   rightMotors.spin(forward, 5, percent);
//   while (abs(imu.rotation(rotationUnits::deg)) < 360)
//   {
//     vex::wait(50, msec);
//     printf("%f \n", imu.rotation(deg));
//   }
//   leftMotors.stop(hold);
//   rightMotors.stop(hold);
//   printf("360: %f \n", topLeftBack.position(rotationUnits::raw));
//   float rpd = topLeftBack.position(rotationUnits::raw) / 360;
//   printf("rpd: %f \n", rpd);
// }

void autonomous(void)
{
  ReCalibrateGyro();
  blocker.spinTo(400, deg, true);
  arm.spinTo(180, deg, true);
  blocker.spinTo(0, deg, true);

  int repeat = 22;
  while (repeat > 0)
  {
    repeat--;
    rotate(360, CW, 30);
  }

  blocker.spinTo(400, deg, true);
  arm.spinTo(0, deg, true);
  blocker.spinTo(0, deg, true);

  rotate(225 - imu.heading(), CW, 30);
  move(12, reverse, 30);
  rotate(45, CW, 30);
  move(80, reverse, 30);
  rotate(45, CW, 30);
  move(34, reverse, 30);
  rotate(45, CW, 30);
  move(24, reverse, 100);
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
