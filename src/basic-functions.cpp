#include "vex.h"
#include "robot-config.h"
#include "basic-functions.h"
#include "parameters.h"
#include "PID.h"
#include "my-timer.h"
#include "ezize.h"
#include "iostream"
using namespace std;

bool autonChecker = false;

void autonFlipper(bool _checker) { autonChecker = _checker; }

float abbs(float x) { return x >= 0 ? x : -x; }

float deg2rad(float deg) { return deg / 180.0 * PI; }

float rad2deg(float rad) { return rad / PI * 180.0; }

float sqrf(float x) { return x * x; }

int sign(float _input) {
  if (_input > 0) return 1;
  else if (_input < 0) return -1;
  else return 0;
}

float deg2range(float _degree){
  int _cir = int(_degree / 360);
  _degree -= _cir * 360;
  if (_degree > 180.0) _degree -= 360;
  if (_degree < -180.0) _degree += 360;
  return _degree;
}

void moveLeft(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseLM.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseLB.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void moveLeftVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

void lockLeft(void) {
  // locks all motors on left side of base
  Motor_BaseLF.stop(vex::brakeType::hold);
  Motor_BaseLM.stop(vex::brakeType::hold);
  Motor_BaseLB.stop(vex::brakeType::hold);
}

void unlockLeft(void) {
  // unlocks all motors on left side of base
  Motor_BaseLF.stop(vex::brakeType::coast);
  Motor_BaseLM.stop(vex::brakeType::coast);
  Motor_BaseLB.stop(vex::brakeType::coast);
}

void moveRight(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseRM.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseRB.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void moveRightVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int) _input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int) _input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

void lockRight(void) {
  // locks all motors on right side of base
  Motor_BaseRF.stop(vex::brakeType::hold);
  Motor_BaseRM.stop(vex::brakeType::hold);
  Motor_BaseRB.stop(vex::brakeType::hold);
}

void unlockRight(void) {
  // unlocks all motors on right side of base
  Motor_BaseRF.stop(vex::brakeType::coast);
  Motor_BaseRM.stop(vex::brakeType::coast);
  Motor_BaseRB.stop(vex::brakeType::coast);
}

void moveForward(float _input) {
  // move forward with _input% power
  moveLeft(_input);
  moveRight(_input);
}

void moveForwardVel(float _input) {
  // move forward with _input% speed
  moveLeftVel(_input);
  moveRightVel(_input);
}

void moveClockwise(float _input) {
  // rotate clockwise with _input% power
  moveLeft(_input);
  moveRight(-_input);
}

void lockBase(void) {
  // lock the base
  lockLeft();
  lockRight();
}

void unlockBase(void) {
  // unlock the base
  unlockLeft();
  unlockRight();
}

float getCrtVel(){
  return (Motor_BaseLM.velocity(pct) + Motor_BaseRM.velocity(pct)) / 2;
}

static volatile float _leftPosLast = 0, _rightPosLast = 0;

float getLeftPos() {
  // return the position of left side of base (mm from last reset position) according to encoder value
  return (Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)+Motor_BaseLB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (3.0*360) - _leftPosLast; // return mm
}

float getRightPos() {
  // return the position of right side of base (mm from last reset position) according to encoder value
  return (Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)+Motor_BaseRB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (3.0*360) - _rightPosLast; // return mm
  //return 0;
}

float getForwardPos() {
  // return the vertical position of base (mm from last reset position) according to encoder value
  return (getLeftPos() + getRightPos()) / 2;
}

void resetLeftPos() {
  // reset encoder on the left side of the base
  _leftPosLast = (Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)+Motor_BaseLB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (3.0*360);
}

void resetRightPos() {
  // reset encoder on the right side of the base
  _rightPosLast = (Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)+Motor_BaseRB.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (3.0*360);
}

void resetForwardPos() {
  // reset encoders on both side of the base
  resetLeftPos();
  resetRightPos();
}

static float headingOffset = 0;

float getHeading() {
  // return the heading angle of robot in deg (+360 after a full clockwise turn, -360 after a counter-clockwise turn) 
  return IMU.rotation() * kGyro + headingOffset;
}

void resetHeading() {
  IMU.resetRotation();
  headingOffset = 0;
}

void resetHeading(float _offset) {
  // reset current heading angle of robot 
  IMU.resetRotation();
  headingOffset = _offset;
}

float getPitch() {
  // return the pitch angle of robot in deg
  return IMU.pitch();
}

void setPistonRight(bool _input) {
  // set ExpansionRIGHT piston accordingly
  if (_input) PistonRight.off();
  else PistonRight.on();
}

void setPistonLeft(bool _input) {
  // set ExpansionLEFT piston accordingly
  if (_input) PistonLeft.off();
  else PistonLeft.on();
}

void setPistonE(bool _input) {
  // set Both Expansion piston accordingly
  setPistonRight(_input);
  setPistonLeft(_input);
}

void setPistonHook(bool _input) {
  // set Expansion piston accordingly
  if (_input)  PistonHook.off();
  else PistonHook.on();
}

void setPistonLIFT(bool _input) {
  // extern LIFT1
  if (_input) {
    PistonLIFT1.open();
    PistonLIFT2.open();
  }

  else {
    PistonLIFT1.close();
    PistonLIFT2.close();
    }
}

float intake_speed = 0;

void setIntakeSpeed(float _input){
  intake_speed = _input;
}

void intake() {
  while(true){
    Motor_Intake1.spin(directionType::fwd, (int)130 * intake_speed, voltageUnits::mV);
    this_thread::sleep_for(1);
  }
}

static int Hstep = 1;
void clearhighLiftStep(){
  Hstep = 1;
}
static int Lstep = 1;
void clearLowLiftStep(){
  Lstep = 1;
}

void autoLowLift(){
  if (Lstep == 1){
    moveLeft(50);
    moveRight(60);
  }
  else if (Lstep == 2){
    moveLeft(25);
    moveRight(60);
  }
  else if (Lstep == 3){
    moveLeft(25);
    moveRight(60);
    this_thread::sleep_for(100);
    lockBase();
    this_thread::sleep_for(100);
    unlockBase();
    Lstep = 4;
  }
  if (IMU.roll() > 13) Lstep = 2;
  else if (Lstep == 2 && IMU.roll() < 2) Lstep = 3;
}

void autohighLift(){
  if (Hstep == 1){
    int Ch1 = abbs(C1) < Joystick_LowerDeadzone ? 0 : C1;
    int Ch3 = abbs(C3) < Joystick_LowerDeadzone ? 0 : C3;
    moveLeft(Ch3 + 2* Ch1);
    moveRight(Ch3 - 2* Ch1);
  }
  else if (Hstep == 2){
    setPistonLIFT(0);
  }
  else if (Hstep == 3){
    moveLeft(0);
    moveRight(0);
    this_thread::sleep_for(100);
    lockBase();
    this_thread::sleep_for(100);
    unlockBase();
    Hstep = 4;
  }
  if (IMU.roll() > 20) Hstep = 2;
  else if (Hstep == 2 && IMU.roll() < 5) Hstep = 3;
}

// ---------- cataStatus --------- 
// 1 = ready to shoot   2 = shooting   
// 5 = switch cata height  4 = initial
// 0 = pulling down to preshoot position
int cataMode = 1;
int cataStatus = 0;
//double ready_Pos = Motor_Cata1.position(deg);
//int intake_Pos = 0;
//int init_Pos = 0;

void setCataStatus(int status, int mode){
  cataStatus = status;
  if (mode != 2) cataMode = mode;
}

int getCataStatus(){
  return cataStatus;
}
  

void catapult(){
  MyTimer cataTimer;
  double ready_Pos = Motor_Cata1.position(deg);
  while(true){
    if (cataStatus == 0){
      while(1){
        if (limit1.PRESSED){
          Motor_Cata1.stop(hold);
          setCataStatus(1);
          Motor_Cata1.resetPosition();
          cataMode = 1;
          break;
        }else{
          Motor_Cata1.setVelocity(45, percent);
          Motor_Cata1.spin(fwd);
          ready_Pos = -2;
        }
      }
      wait(500, msec);
      cataTimer.reset();
      Motor_Cata1.setVelocity(127, percent);
      while (Motor_Cata1.position(deg) > ready_Pos - 250 && cataTimer.getTime() < 1500){
        Motor_Cata1.spinToPosition(ready_Pos - 250, deg, false);
      }
      setCataStatus(1);
    }
    if (cataStatus == 1){
      Motor_Cata1.stop(hold);
    }
    else if (cataStatus == 2){ // R1
      cataTimer.reset();
      Motor_Cata1.setVelocity(127,percent);
      while (Motor_Cata1.position(deg) < ready_Pos + 90 && cataTimer.getTime() < 200){
        Motor_Cata1.spinToPosition(ready_Pos + 90, deg, false);
      }
      this_thread::sleep_for(200);
      Motor_Cata1.setVelocity(127,percent);
      if (cataMode != 1){
        while (Motor_Cata1.position(deg) < ready_Pos + 360 - INTAKEMIDPOS && cataTimer.getTime() < 1500){
          Motor_Cata1.spinToPosition(ready_Pos + 360 - INTAKEMIDPOS, deg, false);
        }
      }
      else{
        while (Motor_Cata1.position(deg) < ready_Pos + 360 && cataTimer.getTime() < 1500){
          Motor_Cata1.spinToPosition(ready_Pos + 360, deg, false);
        }
      }
      if (cataTimer.getTime() < 1500) ready_Pos = ready_Pos + 360;
      setCataStatus(1);
    }
    else if (cataStatus == 4){ // Y
      if (limit1.PRESSED){
        Motor_Cata1.stop(hold);
        setCataStatus(1);
        Motor_Cata1.resetPosition();
        cataMode = 1;
      }else{
        Motor_Cata1.setVelocity(45, percent);
        Motor_Cata1.spin(fwd);
        ready_Pos = -2;
      }
    }
    else if (cataStatus == 5){ // R2
      if (cataMode == 1){
        cataTimer.reset();
        Motor_Cata1.setVelocity(127, percent);
        while (Motor_Cata1.position(deg) != abbs(ready_Pos - INTAKEMIDPOS) && cataTimer.getTime() < 500){
          Motor_Cata1.spinToPosition(ready_Pos - INTAKEMIDPOS, deg);
        }
        cataMode = 0;
        setCataStatus(1);
      }
      else{
        cataTimer.reset();
        Motor_Cata1.setVelocity(127, percent);
        while (Motor_Cata1.position(deg) < ready_Pos && cataTimer.getTime() < 500){
          Motor_Cata1.spinToPosition(ready_Pos, deg);
        }
        cataMode = 1;
        setCataStatus(1);
      }
    }
    else if (cataStatus == 6){ // X
      cataTimer.reset();
      Motor_Cata1.setVelocity(127, percent);
      while (Motor_Cata1.position(deg) > ready_Pos - 250 && cataTimer.getTime() < 1500){
        Motor_Cata1.spinToPosition(ready_Pos - 250, deg, false);
      }
      setCataStatus(1);
    }
    
    this_thread::sleep_for(2);
  }
}