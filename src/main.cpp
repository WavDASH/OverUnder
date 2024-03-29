/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Motor18              motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ezize.h"
#include "basic-functions.h"
#include "parameters.h"
#include "autonomous.h"
#include "my-timer.h"
#include "iostream"
#include "GPS.h"
#include "skill.h"

using namespace std;
using namespace vex;

// A global instance of competition
competition Competition;

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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
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

int auton_choose = 5;
bool autolowlift = 0;
bool autohighlift = 0;
bool LIFTstatus = 0;
bool firstTime = 1;


void autonomous(void) {
  runAuton(auton_choose);
  // runSkill();
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

  int print_i = 0;
  int autocollide = 0;
  bool RightPressed = 0;
  bool LEFTPressed = 0;
  bool BAPressed = 0;
  bool BBPressed = 0;
  bool L2Pressed = 0;
  bool BXPressed = 0;
  bool extensionStatus = 0;
  float m_degree = Motor_Cata1.position(deg);

  while(true) {

    int Ch1 = abbs(C1) < Joystick_LowerDeadzone ? 0 : C1;
    int Ch2 = abbs(C2) < Joystick_LowerDeadzone ? 0 : C2;
    int Ch3 = abbs(C3) < Joystick_LowerDeadzone ? 0 : C3;
    // int Ch4 = abbs(C4) < Joystick_LowerDeadzone ? 0 : C4;
    if (sign(Ch3) == sign(Ch2) && abbs(Ch2) > 80 && abbs(Ch3) > 80 && abbs(Ch2) > (abbs(Ch1) + 40)) autocollide = sign(Ch2);
    else autocollide = 0;
    
    if(LEFT && !LEFTPressed){  // Auto low lift
      setPistonHook(1);
      autolowlift = !autolowlift;
      clearLowLiftStep();
    }
    LEFTPressed = LEFT;

    #ifdef AUTOLIFT
      if(BB && !BBPressed){   // Auto high lift
      LIFTstatus=!LIFTstatus;
        setPistonLIFT(1);
        clearhighLiftStep();
        autohighlift = !autohighlift;
    }
    BBPressed = BB;

    #endif

    #ifdef USERDRIVELIFT
      if(BB && !BBPressed){   //  high lift
      LIFTstatus=!LIFTstatus;
        setPistonLIFT(LIFTstatus);
      }
      BBPressed = BB;

    #endif
    
    if (autolowlift == 1) autoLowLift();
    else if (autohighlift == 1) autohighLift();
    else if (autocollide == 0) {
      //unlockBase();
      moveLeft(Ch3 + 2* Ch1);
      moveRight(Ch3 - 2* Ch1);
    } 
    else {
      moveLeft(autocollide * 15);
      moveRight(autocollide * 15);
    }
        
    if (R1) setIntakeSpeed(100);
    else if (R2) setIntakeSpeed(-100);
    else setIntakeSpeed(0);

    if (BY) setCataStatus(4);
    if (L1) setCataStatus(2);
    if (L2 && !L2Pressed) setCataStatus(5);
    L2Pressed = L2;
    if (BX && !BXPressed) setCataStatus(6);
    BXPressed = BX;

    if (BA && !BAPressed){
      extensionStatus = !extensionStatus;
      setPistonRight(extensionStatus);
      setPistonLeft(extensionStatus);
    }
    BAPressed = BA;
    BBPressed = BB;

    // only when down and R2 are both pressed we run auton
    if (DOWN) runAuton(auton_choose);
    if (DOWN && R2) runSkill();

    if (RIGHT && !RightPressed) auton_choose = ((auton_choose + 1) - 1) % 5 + 1;
    RightPressed = RIGHT;

    m_degree = Motor_Cata1.position(deg) / 3;
    if (print_i == 0){
      // Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Heading: %.1f                             ", getHeading());
    
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("ForwardPosition: %.1f                     ", getForwardPos());

      Brain.Screen.setCursor(7, 1);
      Brain.Screen.print(getCataStatus());

      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("m_degree: %.1f                     ", m_degree);

      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Auton choose: %d", auton_choose);
    }
    print_i += 1;
    print_i %= 100;

    this_thread::sleep_for(5);
    
  }
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
  wait(1000, msec);
  setPistonRight(false);
  setPistonLeft(false);
  setPistonHook(false);
  IMU.startCalibration();
  while (IMU.isCalibrating()) {
  }
  wait(1000, msec);

  thread Intake(intake);
  thread Catapult(catapult);
  thread GPSPosition(MyGpsPos);
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) wait(5, msec);
}
