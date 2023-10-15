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

void setPistonE1(bool _input) {
  // set ExpansionRIGHT piston accordingly
  if (_input) PistonE1.off();
  else PistonE1.on();
}

void setPistonE2(bool _input) {
  // set ExpansionLEFT piston accordingly
  if (_input) PistonE2.off();
  else PistonE2.on();
}

void setPistonE(bool _input) {
  // set Both Expansion piston accordingly
  setPistonE1(_input);
  setPistonE2(_input);
}

void setPistonHook(bool _input) {
  // set Expansion piston accordingly
  if (_input)  PistonHook.off();
  else PistonHook.on();
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

static int step = 1;
void clearLowLiftStep(){
  step = 1;
}

void autoLowLift(){
  if (step == 1){
    moveLeft(50);
    moveRight(60);
  }
  else if (step == 2){
    moveLeft(25);
    moveRight(60);
  }
  else if (step == 3){
    moveLeft(25);
    moveRight(60);
    this_thread::sleep_for(100);
    lockBase();
    this_thread::sleep_for(100);
    unlockBase();
    step = 4;
  }
  if (IMU.roll() > 13) step = 2;
  else if (step == 2 && IMU.roll() < 2) step = 3;
}


// ---------- cataStatus --------- 
// 1 = ready to shoot   2 = shooting   
// 5 = switch cata height  4 = initial
// 0 = pulling down to preshoot position
int cataMode = 1;
int cataStatus = 4;

void setCataStatus(int status, int mode){
  cataStatus = status;
  if (mode != 2) cataMode = mode;
}

int getCataStatus(){
  return cataStatus;
}

  
void catapult(){
  auto cataPID = PID();
  cataPID.setCoefficient(5, 0, 0);
  cataPID.setTarget(0);
  cataPID.setIMax(25);
  cataPID.setIRange(10);
  cataPID.setErrorTolerance(2);
  cataPID.setDTolerance(20);
  cataPID.setJumpTime(50);
  double vel = 0;
  double ready_Pos = Motor_Cata1.position(deg);

  while(true){
    if (cataStatus == 1){
      Motor_Cata1.stop(hold);
    }
    else if (cataStatus == 2){ // R1
      Motor_Cata1.setVelocity(127,percent);
      Motor_Cata1.spinToPosition(ready_Pos + 90, deg);
      wait(200, msec);
      setCataStatus(0);
    }
    else if (cataStatus == 0){ // AFTER 2
      Motor_Cata1.setVelocity(127,percent);
      Motor_Cata1.spinToPosition(ready_Pos + 360, deg);
      setCataStatus(1);
      ready_Pos = ready_Pos + 360;
    }
    else if (cataStatus == 4){ // Y
      if (limit1.PRESSED){
        Motor_Cata1.stop(hold);
        setCataStatus(1);
        Motor_Cata1.resetPosition();

      }else{
        Motor_Cata1.setVelocity(45, percent);
        Motor_Cata1.spin(fwd);
        ready_Pos = -2;
      }
    }
    else if (cataStatus == 5){ // R2
      if (cataMode == 1){
        Motor_Cata1.setVelocity(127, percent);
        Motor_Cata1.spinToPosition(ready_Pos - 55, deg);
        wait(200, msec);
        cataMode = 0;
        setCataStatus(1);
      }
      else if (cataMode == 0){
        Motor_Cata1.setVelocity(127, percent);
        Motor_Cata1.spinToPosition(ready_Pos, deg);
        wait(200, msec);
        cataMode = 1;
        setCataStatus(1);
      }
    }
    else if (cataStatus == 6){ // X
      Motor_Cata1.setVelocity(127, percent);
      Motor_Cata1.spinToPosition(ready_Pos - 250, deg);
      wait(200, msec);
      setCataStatus(1);
    }
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("vel: %.1f                             ", vel);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("pos: %.1f                             ", Motor_Cata1.position(degrees));
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("pitch: %.1f                             ", IMU.orientation(pitch, degrees));
    this_thread::sleep_for(2);
  }
  
}