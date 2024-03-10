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
  imu.calibrate();
  while (imu.isCalibrating())
  {
    vex::wait(15, msec);
  }
}

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

  float revs = ((((10.5 * 3.14) / 360) * degrees) / 6.28) * (1 - (2 * int(direction)));
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

  leftMotors.stop(hold);
  rightMotors.stop(hold);
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
  imu.calibrate();
  while (imu.isCalibrating())
  {
    vex::wait(15, msec);
  }
  move(1.75 * 24, fwd, 30);
  printf("done moving\n");
  vex::wait(500, msec);
  rotate(45, CW, 20);
  printf("done rotating\n");
  vex::wait(500, msec);
  intake.spin(forward);
  move(10, fwd, 30);
  wings.open();
  vex::wait(500, msec);
  rotate(45, CW, 30);
  vex::wait(500, msec);
  intake.spin(reverse);
  vex::wait(500, msec);
  move(2 * 24, fwd, 100);
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
  rightMotors.stop(coast);
  leftMotors.stop(coast);

  float moveSpeed = 1;
  float spinSpeed = .6;
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    if (Controller1.ButtonY.pressing())
    {
      wings.open();
    }
    if (Controller1.ButtonRight.pressing())
    {
      wings.close();
    }

    if (Controller1.ButtonR2.pressing())
    {
      intake.spin(reverse, 100, percent);
    }
    else if (Controller1.ButtonR1.pressing())
    {
      intake.spin(forward, 100, percent);
    }
    else
    {
      intake.stop(brake);
    }

    if (Controller1.ButtonA.pressing())
    {
      arm.spin(forward, 30, percent);
    }
    else if (Controller1.ButtonB.pressing())
    {
      arm.spin(reverse, 30, percent);
    }
    else
    {
      arm.stop(brakeType::hold);
    }

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print(imu.angle());

    // if (Controller1.ButtonDown.pressing())
    // {
    //   rightMotors.setVelocity(35, percent);
    //   leftMotors.stop(coast);
    //   rightMotors.spinTo(-300,rotationUnits::raw,true);
    //   vex::wait(.5, sec);
    //   rightMotors.spinTo(0,rotationUnits::raw,true);
    // }

    leftMotors.spin(forward,
                    (-Controller1.Axis3.position() * moveSpeed - Controller1.Axis1.position() * spinSpeed),
                    pct);

    rightMotors.spin(forward,
                     (Controller1.Axis3.position() * moveSpeed - Controller1.Axis1.position() * spinSpeed),
                     pct);

    if (Controller1.ButtonL1.pressing())
    {
      blocker.spinTo(500, degrees, false);
    }
    else if (Controller1.ButtonL2.pressing())
    {
      blocker.spinTo(0, degrees, false);
    }

    vex::wait(20, msec); // Sleep the task for a short amount of time to
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
    vex::wait(100, msec);
  }
}
