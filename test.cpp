int cataPos = 0;
void catapult(){
  MyTimer cataTimer;
  while(1){
    if (cataStatus == -1){ // go to ready position
      cataTimer.reset();
      Motor_Cata1.setVelocity(127,percent);
      while (Motor_Cata1.position(deg) < ready_Pos - intake_Pos - init_Pos && cataTimer.getTime() < 1500){
        Motor_Cata1.spinToPosition(ready_Pos - intake_Pos - init_Pos, deg, false);
      }
      setCataStatus(5);
    }
    else if (cataStatus == 1){ //shoot
      cataTimer.reset();
      Motor_Cata1.setVelocity(127,percent);
      while (Motor_Cata1.position(deg) < ready_Pos + 90 && cataTimer.getTime() <1000){
        Motor_Cata1.spinToPosition(ready_Pos + 90, deg, false);
      }
      if (cataTimer.getTime() < 1000){
        ready_Pos += 360;
        cataMode = 1;
      } 
      this_thread::sleep_for(200);
      setCataStatus(0);
    }
    else if (cataStatus == 2){ //switch cata height
    init_Pos = 0;
      if (cataMode == 1){
        intake_Pos = INTAKEMIDPOS;
        cataMode = 0;
        setCataStatus(0);
      }
      else if (cataMode == 0){
        intake_Pos = 0;
        cataMode = 1;
        setCataStatus(0);
      }
    }
    else if (cataStatus == 3){ // go to initial height
      init_Pos = 250;
      intake_Pos = 0;
      cataMode = 1;
      setCataStatus(0);
    }
    else if (cataStatus == 4){ //initialize
      if (limit1.PRESSED){
        Motor_Cata1.stop(hold);
        setCataStatus(5);
        Motor_Cata1.resetPosition();
        ready_Pos = Motor_Cata1.position(deg);
        cataMode = 1;
        //break;
      }else{
        Motor_Cata1.setVelocity(45, percent);
        Motor_Cata1.spin(fwd);
        ready_Pos = -2;
      }
    }
    else if (cataStatus == 5){ //hold
      Motor_Cata1.stop(hold);
    }
  }
  this_thread::sleep_for(2);
}