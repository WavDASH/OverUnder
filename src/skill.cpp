#include "skill.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"

void skillInit() {
  setPistonLeft(false);
  setPistonRight(false);
  MyGps.resetForwardPosGps();
}

void skill(){
  MyTimer autotimer;
  autotimer.reset();
}

void runSkill(){
    skillInit();
    skill();
}
