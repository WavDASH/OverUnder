#include "autonomous.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"
#include "ezize.h"

void autonInit(void) {
  setPistonE1(false);
  setPistonE2(false);
  resetHeading();
  resetForwardPos();
  MyGps.resetForwardPosGps();
}


void auton_near_1(){
  MyTimer autotimer;
  autotimer.reset();
  setIntakeSpeed(100);
  setCataStatus(5, 0);
  softStartTimerForward(0, 80, 200);
  posForwardAbsWithHeading(80, 1000, 0);
  PIDPosForwardAbs(1180);
  PIDPosForwardAbs(670);
  PIDAngleRotateAbs(-135, 2.5, 0.05, 5, 1.5);
  setCataStatus(2, 0); // shoot
  this_thread::sleep_for(200);

  PIDAngleRotateAbs(-160, 2, 0.05, 4);
  PIDPosForwardAbs(500);
  posForwardAbsWithHeading(90, 0, -160);
  setIntakeSpeed(0);
  //PIDPosForwardAbs(-600);
  PIDAngleRotateAbs(-80, 2, 0.05, 4);
  setIntakeSpeed(-100);
  posForwardAbsWithHeading(50, 400, -80);
  voltageForward(70);
  this_thread::sleep_for(200);
  lockBase();
  this_thread::sleep_for(50);

  setIntakeSpeed(0);
  PIDPosForwardAbs(0);
  PIDAngleRotateAbs(40, 2, 0.05, 5);
  posForwardAbsWithHeading(50, -1300, 20);
  voltageForward(-10);
  this_thread::sleep_for(400);

  setPistonHook(1);
  this_thread::sleep_for(100);
  PIDAngleRotateAbs(-80, 2, 0.05, 5);
  setPistonHook(0);
  PIDPosForwardAbs(-600);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_near_2(){
  MyTimer autotimer;
  autotimer.reset( );
  setIntakeSpeed(0);
  setPistonHook(1);
  this_thread::sleep_for(300);
  PIDAngleRotateAbs(-90, 1.5, 0.05, 5);
  setPistonHook(0);

  PIDPosForwardAbs(650);
  //PIDPosCurveAbs(500,300,5);
  PIDAngleRotateAbs(-45, 1.1, 0.05, 5);
  setIntakeSpeed(-100);
  voltageForward(10);
  this_thread::sleep_for(300);
  voltageForward(90);
  this_thread::sleep_for(400);

  setIntakeSpeed(0);
  setCataStatus(5,0);
  PIDPosForwardAbs(0);
  PIDAngleRotateAbs(-90,1.1,0.05,5);
  PIDPosForwardAbs(-570);
  PIDAngleRotateAbs(-135,1.1,0.05,5);
  PIDPosForwardAbs(-895);
  //PIDPosCurveAbs(-1600,-1100,5);
  
  this_thread::sleep_for(100);


  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_1(){
  MyTimer autotimer;
  autotimer.reset();
  setIntakeSpeed(100);//
  setCataStatus(5, 1);
  softStartTimerForward(0, 80, 200);
  posForwardAbsWithHeading(80, 1100, 0);
  PIDPosForwardAbs(1400);
  PIDAngleRotateAbs(105, 2, 0.05, 5);
  setIntakeSpeed(0);
  posForwardAbsWithHeading(80, 150, 105);
  setIntakeSpeed(-100);
  voltageForward(10);
  this_thread::sleep_for(300);
  voltageForward(60);
  this_thread::sleep_for(300);

  PIDPosForwardAbs(0);
  PIDAngleRotateAbs(-75, 1.5, 0.05, 10);
  setIntakeSpeed(100);
  //PIDPosForwardAbs(330);
  voltageForward(50);
  this_thread::sleep_for(600);
  PIDPosForwardAbs(0);
  setIntakeSpeed(0);
  PIDAngleRotateAbs(100, 1.5, 0.05, 10);
  posForwardAbsWithHeading(80, 150, 100);
  setIntakeSpeed(-100);
  voltageForward(20);
  this_thread::sleep_for(350);
  voltageForward(70);
  this_thread::sleep_for(300);

  PIDPosForwardAbs(0);
  PIDAngleRotateAbs(-165, 1.5, 0.05, 10);
  PIDPosForwardAbs(550);
  PIDAngleRotateAbs(-75, 1.5, 0.05, 10);
  setIntakeSpeed(100);
  //PIDPosForwardAbs(350);
  voltageForward(50);
  this_thread::sleep_for(620);
  PIDPosForwardAbs(10);
  setIntakeSpeed(0);
  PIDAngleRotateAbs(65, 1.5, 0.05, 10);
  posForwardAbsWithHeading(80, 150, 65);
  setIntakeSpeed(-100);
  voltageForward(20);
  this_thread::sleep_for(450);
  voltageForward(70);
  this_thread::sleep_for(400);
  PIDPosForwardAbs(0);
  setIntakeSpeed(0);
  //setPistonE(1);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_2(){
  MyTimer autotimer;
  autotimer.reset();

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void test(){
  MyTimer autotimer;
  autotimer.reset();

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}


// #ifdef ROBOT1
void runAuton(int auton_choose) {
  
  setAutonMode();
  autonFlipper(true);
  autonInit();

  if (auton_choose == 1) auton_near_1();        // 1+2 near
  else if (auton_choose == 2) auton_near_2();   //
  else if (auton_choose == 3) auton_far_1();    //
  else if (auton_choose == 4) auton_far_2();
}
// #endif
