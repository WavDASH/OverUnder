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


void auton_near_1(){//角球&预装&抢中球
  MyTimer autotimer;
  autotimer.reset();
  setCataStatus(5, 1);
  PIDPosForwardAbs(-300);
  setPistonRight(1);//开右板
  posForwardAbsWithHeading(60,-10,0);
  PIDAngleRotateAbs(-43,2,0.05,10,1);//转向高挂杆
  PIDPosForwardAbs(930);//推走中立球
  setPistonRight(0);//
  setIntakeSpeed(0);
  posForwardAbsWithHeading(80,150,-45);//回到导入区
  PIDAngleRotateAbs(0,2,0.05,10,1);//平行导入杆
  posForwardAbsWithHeading(80,-250,0);//退到围板边
  PIDPosCurveAbs(-10,-350,2);//车背正对网
  voltageForward(-70);//送入预装球
  this_thread::sleep_for(400);

  PIDAngleRotateAbs(-45,3,0.05,10,2);//背对围板
  posForwardAbsWithHeading(80,430,-45);//直行
  setCataStatus(5,0);//降下弹射板
  PIDPosCurveAbs(10,550,2);//走弧线朝向中间球
  setIntakeSpeed(100);//吸球
  posForwardAbsWithHeading(80,500,-125);//朝向-125°直线
  PIDPosForwardAbs(700);//吸到中立球
  this_thread::sleep_for(200);//等球进入发射板
  PIDAngleRotateAbs(45,2,0.05,10,1);//调整发射角度
  lockBase();
  setCataStatus(2,1);//发射
  this_thread::sleep_for(200);
  unlockBase();
  PIDAngleRotateAbs(-225,1.5,0.08,10,1.5);//正对网
  setCataStatus(5,1);
  setPistonE(1);
  lockBase();//锁住底盘

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_near_2(){//完成任务
  MyTimer autotimer;
  autotimer.reset( );
  
  setCataStatus(5, 1);
  PIDPosForwardAbs(-300);//后退
  setPistonRight(1);//开右板
  posForwardAbsWithHeading(40,-20,0);//推出角落球
  setPistonRight(0);//关右板
  PIDPosCurveAbs(-250, -610, 5);
  voltageForward(-20);
  this_thread::sleep_for(300);
  voltageForward(-60);
  this_thread::sleep_for(300);
  posForwardAbsWithHeading(40,0,0);
  setPistonRight(1);
  PIDPosForwardAbs(590);
  PIDAngleRotateAbs(-45,2,0.05,10,1);
  setIntakeSpeed(-100);
  PIDPosForwardAbs(790);
  setPistonRight(0);
  this_thread::sleep_for(300);
  setIntakeSpeed(0);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_1(){//中间三颗球和预装
  MyTimer autotimer;
  autotimer.reset();
  setCataStatus(5, 1);
  softStartTimerForward(0, 80, 200);
  setIntakeSpeed(100);
  posForwardAbsWithHeading(80, 1100, 0);
  PIDPosForwardAbs(1350); // 吸到第一颗球
  setIntakeSpeed(0);
  PIDAngleRotateAbs(105, 2, 0.05, 18,1.5); // 转到goal
  setIntakeSpeed(-100); // 吐球
  //posForwardAbsWithHeading(80, 150, 105); // 送球
  voltageForward(15);// 送球
  this_thread::sleep_for(200);// 送球
  //setIntakeSpeed(-100);
  voltageForward(70);// 送球
  this_thread::sleep_for(420);// 送球
  //PIDPosForwardAbs(200);

  PIDPosForwardAbs(150); // 第一颗球后退到中间
  PIDAngleRotateAbs(-75, 2, 0.05, 18,1.5); // 朝向第二颗球
  setIntakeSpeed(100); // 吸球
  posForwardAbsWithHeading(80, 350, -75); // 前进
  voltageForward(40); //吸到第二颗球
  this_thread::sleep_for(300); //吸到第二颗球
  setIntakeSpeed(30); // 吸球
  PIDPosForwardAbs(50); // 第二颗球后退到中间
  PIDAngleRotateAbs(120, 2, 0.05, 18, 1.5); //朝向goal
  setIntakeSpeed(-100); // 吐球
  posForwardAbsWithHeading(80, 150, 120); 
  voltageForward(70); //送球
  this_thread::sleep_for(400);
  //PIDPosForwardAbs(200);
  setIntakeSpeed(0);
  PIDPosForwardAbs(150); // 送完第二颗球后退到中间
  PIDAngleRotateAbs(-165, 1.5, 0.05, 18,2); //朝向围板
  posForwardAbsWithHeading(80, 50, -165); //速度、距离、角度
  setIntakeSpeed(100);
  PIDPosCurveAbs(500,100,10);//走曲线
  voltageForward(65); // 吸到第三颗球
  this_thread::sleep_for(400); // 吸到第三颗球
  PIDPosForwardAbs(350);
  setIntakeSpeed(20); // 吸第三颗球
  posForwardAbsWithHeading(50,50,-75); // 回到中间
  setPistonRight(1);
  PIDAngleRotateAbs(-188,2,0.08,15,2);
  PIDPosForwardAbs(390);// 朝向预装球
  //setIntakeSpeed(-100); // 吐球
  PIDPosCurveAbs(80,950,10);//左边轮子、右边轮子、误差
  setIntakeSpeed(-100); // 吐球
  this_thread::sleep_for(300);
  voltageForward(100); // 送第三颗球
  this_thread::sleep_for(500);
  PIDPosForwardAbs(0); // 回到中间
  setPistonRight(0);
  setCataStatus(5, 1);
  // setPistonE(1);

  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_2(){//5球
  MyTimer autotimer;
  autotimer.reset();
  setCataStatus(5,1);
  setPistonRight(1);//打开侧板
  this_thread::sleep_for(200);
  setIntakeSpeed(-20);//开启吸球
  PIDPosForwardAbs(300);//前进
  PIDAngleRotateAbs(-30,1.5,0.1,8,1.5);//转向网
  setIntakeSpeed(-100);//吐球
  voltageForward(30);//开向网
  this_thread::sleep_for(200);
  voltageForward(80);//开向网
  this_thread::sleep_for(400);


  PIDPosForwardAbs(150);//后退
  setPistonRight(0);//关闭侧板
  //setCataStatus(6);
  PIDAngleRotateAbs(-108,1.5,0.08,15,1.2);//转向去吸第二颗球
  setIntakeSpeed(100);//开启吸球
  posForwardAbsWithHeading(90, 850, -115); //往球走
  PIDPosForwardAbs(1040);//开向球
  voltageForward(40);//缓慢开向球
  this_thread::sleep_for(280);

  setIntakeSpeed(40);
  PIDAngleRotateAbs(-30,1.5,0.1,10);//转向球
  setPistonLeft(1);//打开侧板，带动第三颗球
  //posForwardAbsWithHeading(70, 250, -25);//开向球
  PIDPosForwardAbs(340);//开向球
  setIntakeSpeed(-50);//开启吐球
  PIDPosCurveAbs(570,120,10);//走曲线
  setIntakeSpeed(-100);
  this_thread::sleep_for(300);
  voltageForward(100);//继续朝网开
  this_thread::sleep_for(300);
  //setPistonLeft(0);//关闭侧板

  PIDPosForwardAbs(50);
  PIDAngleRotateAbs(-133,1.5,0.1,10,1.2);//转向第三颗球
  setIntakeSpeed(100);//开启吸球
  PIDPosForwardAbs(510);//往球走
  voltageForward(30);//继续朝杆开
  this_thread::sleep_for(350);
  setIntakeSpeed(0);
  PIDPosForwardAbs(50);//往后退
  PIDAngleRotateAbs(50,1.5,0.1,10,3);//转向网
  setIntakeSpeed(-100);//吐球
  this_thread::sleep_for(400);
  voltageForward(15);//往前走
  this_thread::sleep_for(200);
  voltageForward(80);//往前走
  this_thread::sleep_for(500);
  setIntakeSpeed(0);
  PIDPosForwardAbs(100);
  setPistonLeft(0);

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
  Controller1.Screen.clearScreen();

  if (auton_choose == 1) auton_near_1();        // 1+2 near
  else if (auton_choose == 2) auton_near_2();   //
  else if (auton_choose == 3) auton_far_1();    //
  else if (auton_choose == 4) auton_far_2();
  else if (auton_choose == 5) test();
}
// #endif