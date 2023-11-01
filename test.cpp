double ready_Pos = Motor_Cata1.position(deg);

  while(true){
    if (cataStatus == -1){
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
      setCataStatus(0);
    }
    else if (cataStatus == 0){ // AFTER 2
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
        while (Motor_Cata1.position(deg)<abbs(ready_Pos - INTAKEMIDPOS) && cataTimer.getTime() < 500){
          Motor_Cata1.spinToPosition(ready_Pos - INTAKEMIDPOS, deg, false);
        }
        cataMode = 0;
        setCataStatus(1);
      }
      else{
        cataTimer.reset();
        Motor_Cata1.setVelocity(127, percent);
        while (Motor_Cata1.position(deg) < ready_Pos && cataTimer.getTime() < 500){
          Motor_Cata1.spinToPosition(ready_Pos, deg, false);
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