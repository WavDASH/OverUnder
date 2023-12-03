#include "autonomous.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"
#include "ezize.h"

void autonInit(void) {
  setPistonRight(false);
  setPistonLeft(false);
  setPistonHook(false);
  resetHeading();
  resetForwardPos();
  MyGps.resetForwardPosGps();
}


void auton_near_1(){
  MyTimer autotimer;
  autotimer.reset();
  
  setCataStatus(5, 1);
  setIntakeSpeed(0);
  setPistonRight(1); //打开右边推球
  PIDPosForwardAbs(1300); //前进并撞走中间的球
  PIDPosForwardAbs(1250); //后退
  PIDAngleRotateAbs(-80,1.1,0.05,5); //左转90度
  setIntakeSpeed(-100); //吐球
  voltageForward(10); //将引入球放入框
  this_thread::sleep_for(200);
  voltageForward(90);
  this_thread::sleep_for(350);
  PIDPosForwardAbs(0);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_near_2(){
  MyTimer autotimer;
  autotimer.reset( );
  
  setCataStatus(5, 1);
  posForwardAbsWithHeading(80, -330, 0);
  PIDAngleRotateAbs(45, 2, 0.05, 3);
  voltageForward(-10);
  this_thread::sleep_for(200);
  voltageForward(-70);
  this_thread::sleep_for(400);
  PIDPosForwardAbs(25);
  PIDAngleRotateAbs(20, 2, 0.05, 3);
  setPistonRight(1);
  PIDPosForwardAbs(280);
  PIDAngleRotateAbs(-30, 2, 0.05, 3);
  posForwardAbsWithHeading(80, 250, 0);
  PIDAngleRotateAbs(-45, 2, 0.05, 3);
  setIntakeSpeed(-100);
  PIDPosForwardAbs(730);
  setIntakeSpeed(0);
  setPistonRight(0);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_1(){
  MyTimer autotimer;
  autotimer.reset();
  setIntakeSpeed(100);//吸入速度百分百
  setCataStatus(5, 1);//5，1 中弹射位置  5，0 低弹射位置
  softStartTimerForward(0, 80, 200);//软启动
  posForwardAbsWithHeading(80, 1200, 0);//走直线 80速度 1200位置 陀螺仪度数
  PIDPosForwardAbs(1400);//pid 走到1400 吸到第一颗球 
  PIDAngleRotateAbs(105, 2, 0.05, 5);//朝向goal （角度 pid kp ki kd）
  setIntakeSpeed(0);
  posForwardAbsWithHeading(80, 150, 105);//走直线
  setIntakeSpeed(-100);
  voltageForward(10);
  this_thread::sleep_for(300);
  voltageForward(60);
  this_thread::sleep_for(300);

  PIDPosForwardAbs(0);
  PIDAngleRotateAbs(-75, 1.5, 0.05, 10);
  setIntakeSpeed(100);
  //PIDPosForwardAbs(330);
  voltageForward(40);
  this_thread::sleep_for(300); // 吸第二颗球
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
  PIDPosForwardAbs(570);
  PIDAngleRotateAbs(-75, 1.5, 0.05, 10);
  setIntakeSpeed(100);
  //PIDPosForwardAbs(350);
  voltageForward(70);
  this_thread::sleep_for(300); // 吸第三颗球
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

  setCataStatus(5, 1);
  setPistonRight(1);//打开侧板
  setIntakeSpeed(20);//开启吸球
  PIDPosForwardAbs(220);//前进
  PIDPosCurveAbs(8,190,5);//走曲线
  setIntakeSpeed(-100);//吐球
  voltageForward(100);//开向网
  this_thread::sleep_for(150);
  voltageForward(70);//开向网
  this_thread::sleep_for(550);
  setPistonRight(0);

  PIDPosForwardAbs(100);//后退
  PIDAngleRotateAbs(-115,1.5,0.08,8,1);//转向去吸第二颗球
  setIntakeSpeed(100);//开启吸球
  posForwardAbsWithHeading(90, 1000, -115); //往球走
  PIDPosForwardAbs(1120);//开向球

  setIntakeSpeed(10);
  PIDAngleRotateAbs(-25,1.5,0.1,10);//转向球
  setPistonLeft(1);//打开侧板
  posForwardAbsWithHeading(70, 200, -25);//开向球
  //setIntakeSpeed(-50);//开启吐球
  PIDPosCurveAbs(400,10,10);//走曲线
  setIntakeSpeed(-100);
  voltageForward(100);//继续朝网开
  this_thread::sleep_for(400);
  setPistonLeft(0);//关闭侧板

  PIDPosForwardAbs(50);
  PIDAngleRotateAbs(-115,1.5,0.1,10,1.2);//转向第三颗球
  setIntakeSpeed(100);//开启吸球
  PIDPosForwardAbs(350);//往球走
  voltageForward(15);//继续朝gan开
  this_thread::sleep_for(200);
  setIntakeSpeed(0);
  PIDPosForwardAbs(50);//往后退
  PIDAngleRotateAbs(50,1.5,0.1,10,3);//转向网
  setIntakeSpeed(-100);//吐球
  voltageForward(15);//往前走
  this_thread::sleep_for(200);
  voltageForward(80);//往前走
  this_thread::sleep_for(500);
  setIntakeSpeed(0);
  PIDPosForwardAbs(100);
  
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void test(){
  MyTimer autotimer;
  autotimer.reset();

  PIDPosForwardAbs(500);
  PIDPosForwardAbs(0);  

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}


// #ifdef ROBOT1
void runAuton(int auton_choose) {
  
  setAutonMode();
  autonFlipper(true);
  autonInit();
  Controller1.Screen.clearScreen();

  if (auton_choose == 1) auton_near_1();        // 1+2 near
  else if (auton_choose == 2) auton_near_2();   //
  else if (auton_choose == 3) auton_far_1();    //
  else if (auton_choose == 4) auton_far_2();
  else if (auton_choose == 5) test();
}
// #endif
