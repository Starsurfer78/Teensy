// motor controller (normal & perimeter tracking), odometry

void Robot::resetMotorFault() {
  /*
    if (digitalRead(pinMotorLeftFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    ShowMessageln(F("Reset motor left fault"));
    }
    if  (digitalRead(pinMotorRightFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    ShowMessageln(F("Reset motor right fault"));
    }
    if (digitalRead(pinMotorMowFault) == LOW) {
    digitalWrite(pinMotorMowEnable, LOW);
    digitalWrite(pinMotorMowEnable, HIGH);
    ShowMessageln(F("Reset motor mow fault"));
    }
  */
}

// sets mower motor actuator
// - ensures that the motor is not switched to 100% too fast (motorMowAccel)
// - ensures that the motor voltage is not higher than motorMowSpeedMaxPwm
void Robot::setMotorMowPWM(int pwm, boolean useAccel) {
  unsigned long TaC = millis() - lastSetMotorMowSpeedTime;    // sampling time in millis
  lastSetMotorMowSpeedTime = millis();
  if (TaC > 1000) TaC = 1;
  //bber13
  if ( (!useAccel)) {  //accel is not use when stop the blade on tilt
    motorMowPWMCurr = pwm;
  } else {
    motorMowPWMCurr += int(TaC) * (pwm - motorMowPWMCurr) / motorMowAccel;
  }
  setActuator(ACT_MOTOR_MOW, min(motorMowSpeedMaxPwm, max(0, motorMowPWMCurr)));
}



// ensures that the motors (and gears) are not switched to 0% (or 100%) too fast (motorAccel)
void Robot::setMotorPWM(int pwmLeft, int pwmRight) {
  int TaC = int(millis() - lastSetMotorSpeedTime);    // sampling time in millis
  lastSetMotorSpeedTime = millis();
  if (TaC > 1000) TaC = 1;

  if (stateCurr != STATE_OFF) {
    /*
      ShowMessage(stateNames[stateCurr]);
      ShowMessage(" Voeux a ");
      ShowMessage (millis());
      ShowMessage(" TaC=");
      ShowMessage (TaC);
      ShowMessage(" Useaccel=");
      ShowMessage (useAccel);
      ShowMessage(" pwmLeft ");
      ShowMessage (pwmLeft);
      ShowMessage(" pwmRight ");
      ShowMessage (pwmRight);

      ShowMessage(" OdoRight ");
      ShowMessage (odometryRight);
      ShowMessage(" OdoLeft ");
      ShowMessageln (odometryLeft);

      ShowMessage ("  motorLeftZeroTimeout : ");
      ShowMessage (motorLeftZeroTimeout);

      ShowMessage(" motorLeftPWMCurr=");
      ShowMessage(motorLeftPWMCurr);
      ShowMessage(" motorRightPWMCurr=");
      ShowMessageln (motorRightPWMCurr);
    */
  }

  // ----- driver protection (avoids driver explosion) ----------
  if ( ((pwmLeft < 0) && (motorLeftPWMCurr > 0)) || ((pwmLeft > 0) && (motorLeftPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      ShowMessage("WARNING PROTECTION ON LEFT MOTOR ");
      ShowMessage("  motorLeftPWMCurr=");
      ShowMessage (motorLeftPWMCurr);
      ShowMessage("  pwmLeft=");
      ShowMessage (pwmLeft);
      ShowMessage(" state ");
      ShowMessageln(stateNames[stateCurr]);
    }
    if (motorLeftZeroTimeout != 0) pwmLeft = motorLeftPWMCurr - motorLeftPWMCurr * ((float)TaC) / 200.0; // reduce speed
  }
  if ( ((pwmRight < 0) && (motorRightPWMCurr > 0)) || ((pwmRight > 0) && (motorRightPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      ShowMessage("WARNING PROTECTION ON RIGHT MOTOR ");
      ShowMessage("  motorRightPWMCurr=");
      ShowMessage (motorRightPWMCurr);
      ShowMessage("  pwmRight=");
      ShowMessage (pwmRight);
      ShowMessage("  On state ");
      ShowMessageln(stateNames[stateCurr]);
    }
    if (motorRightZeroTimeout != 0) pwmRight = motorRightPWMCurr - motorRightPWMCurr * ((float)TaC) / 200.0; // reduce speed
  }

  motorLeftPWMCurr = pwmLeft;
  motorRightPWMCurr = pwmRight;

  if (abs(motorLeftRpmCurr) < 1) motorLeftZeroTimeout = max(0, ((int)(motorLeftZeroTimeout - TaC)) );
  else motorLeftZeroTimeout = 1000;
  if (abs(motorRightRpmCurr) < 1) motorRightZeroTimeout = max(0, ((int)(motorRightZeroTimeout - TaC)) );
  else motorRightZeroTimeout = 1000;

  if (stateCurr != STATE_OFF) {
    /*
      ShowMessage(" result ");
      ShowMessage (millis());
      ShowMessage(" Right/Left ");
      ShowMessage (motorRightPWMCurr);
      ShowMessage(" / ");
      ShowMessageln (motorLeftPWMCurr);
    */
  }

  if (motorLeftSwapDir)  // swap pin polarity?
    setActuator(ACT_MOTOR_LEFT, -motorLeftPWMCurr);
  else
    setActuator(ACT_MOTOR_LEFT, motorLeftPWMCurr);
  if (motorRightSwapDir)   // swap pin polarity?
    setActuator(ACT_MOTOR_RIGHT, -motorRightPWMCurr);
  else
    setActuator(ACT_MOTOR_RIGHT, motorRightPWMCurr);
}



void Robot::OdoRampCompute() { //execute only one time when a new state execution
  //Compute the accel duration (very important for small distance)
  //Compute when you need to brake the 2 wheels to stop at the ODO
  //Compute the estimate duration of the state so can force next state if the mower is stuck
  stateStartOdometryLeft = odometryLeft;
  stateStartOdometryRight = odometryRight;

  motorRightPID.reset();
  PwmRightSpeed = min(motorSpeedMaxPwm, max(-motorSpeedMaxPwm, map(motorRightSpeedRpmSet, -motorSpeedMaxRpm, motorSpeedMaxRpm, -motorSpeedMaxPwm, motorSpeedMaxPwm)));
  PwmLeftSpeed = min(motorSpeedMaxPwm, max(-motorSpeedMaxPwm, map(motorLeftSpeedRpmSet, -motorSpeedMaxRpm, motorSpeedMaxRpm, -motorSpeedMaxPwm, motorSpeedMaxPwm)));
  //try to find when we need to brake the wheel (depend of the distance)

  int  distToMoveLeft;
  int  distToMoveRight;
  distToMoveLeft = abs(stateStartOdometryLeft - stateEndOdometryLeft);
  distToMoveRight = abs(stateStartOdometryRight - stateEndOdometryRight);
 
  //left wheel
  if (distToMoveLeft >= odometryTicksPerRevolution)  {
    OdoStartBrakeLeft =  odometryTicksPerRevolution / 2; //si plus d'1 tour on freine dans la moitie du dernier tour
    SpeedOdoMaxLeft = PwmLeftSpeed; //valeur de vitesse max en fonction de la distance a parcourir
  } else {  // si moins d 1 tour
    if (UseAccelLeft && UseBrakeLeft) { //need 2 ramp
      OdoStartBrakeLeft = distToMoveLeft / 2; //on freine a la moitie de la distance a parcourir
      if (PwmLeftSpeed <= 0) {
        // SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
        SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      } else {
        //SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
        SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
      }
    } else
    { //need 1 ramp
      OdoStartBrakeLeft = distToMoveLeft ; //on freine sur toute la distance a parcourir
      if (PwmLeftSpeed <= 0) SpeedOdoMaxLeft = map(distToMoveLeft , odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxLeft = map(distToMoveLeft , odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
    }
  }

  //right wheel
  if (distToMoveRight >= odometryTicksPerRevolution) { //more than 1 rev
    OdoStartBrakeRight =  odometryTicksPerRevolution / 2;
    SpeedOdoMaxRight = PwmRightSpeed;
  } else {  //if less than 1 rev right wheel
    if (UseAccelRight && UseBrakeRight) {
      OdoStartBrakeRight = distToMoveRight / 2; //on freine a la moitie de la distance a parcourir
      if (PwmRightSpeed <= 0) {
        SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      }
      else {
        SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
      }
    } else
    {
      OdoStartBrakeRight = distToMoveRight ; //on freine sur toute la distance a parcourir
      if (PwmRightSpeed <= 0) SpeedOdoMaxRight = map(distToMoveRight , odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxRight = map(distToMoveRight , odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
    }
  }

  //compute the approximative moving time in millis()
  //Need to compute in 2 times to avoid overflow  !!!!!
  movingTimeLeft = 1000 * distToMoveLeft / motorTickPerSecond ;
  movingTimeLeft = movingTimeLeft * motorSpeedMaxPwm / abs(SpeedOdoMaxLeft);
  //bber500
  if (movingTimeLeft < 4000 ) movingTimeLeft = 4000;
  //for small mouvement need to increase duration
  movingTimeRight = 1000 * distToMoveRight / motorTickPerSecond ;
  movingTimeRight = movingTimeRight * motorSpeedMaxPwm / abs(SpeedOdoMaxRight);
  //bber500 reduce movement shock
  if (movingTimeRight < 4000 ) movingTimeRight = 4000;
  //for small mouvement need to reduce the accel duration
  if (movingTimeLeft >= motorOdoAccel) accelDurationLeft = motorOdoAccel;
  else   accelDurationLeft =  movingTimeLeft / 2;
  if (movingTimeRight >= motorOdoAccel) accelDurationRight = motorOdoAccel;
  else   accelDurationRight =  movingTimeRight / 2;
  if (statusCurr == TESTING) {  //avoid maxduration stop when use test Odo with Pfod
    MaxOdoStateDuration = 30000 + max(movingTimeRight, movingTimeLeft); //add 30 secondes to the max moving duration of the 2 wheels
  } else {
    MaxOdoStateDuration = 3000 + max(movingTimeRight, movingTimeLeft); //add 3 secondes to the max moving duration of the 2 wheels
  }
  //check to set the correct heading
  imuDriveHeading = imu.ypr.yaw / PI * 180; //normal mowing heading

  if (statusCurr == BACK_TO_STATION) {  //possible heading change
    imuDriveHeading = periFindDriveHeading / PI * 180;
  }
  if (statusCurr == REMOTE) {   //possible heading change
    imuDriveHeading = remoteDriveHeading / PI * 180;
  }
  /*
    ShowMessage(" **************** compute  at  ");
    ShowMessageln(millis());
    ShowMessage(" UseAccelRight ");
    ShowMessage(UseAccelRight);
    ShowMessage(" UseBrakeRight ");
    ShowMessage(UseBrakeRight);
    ShowMessage(" UseAccelLeft ");
    ShowMessage(UseAccelLeft);
    ShowMessage(" UseBrakeLeft ");
    ShowMessage(UseBrakeLeft);
    ShowMessage(" distToMoveLeft ");
    ShowMessage(distToMoveLeft);
    ShowMessage(" movingTimeLeft ");
    ShowMessage(movingTimeLeft);
    ShowMessage("ms movingTimeRight ");
    ShowMessageln(movingTimeRight);
    ShowMessage("accelDurationLeft ");
    ShowMessage(accelDurationLeft);
    ShowMessage("ms accelDurationRight ");
    ShowMessageln(accelDurationRight);

    ShowMessage (F(stateNames[stateNext]));
    ShowMessage(" RightSpeedRpmSet ");
    ShowMessage(motorRightSpeedRpmSet);
    ShowMessage("  PwmRightSpeed ");
    ShowMessage(PwmRightSpeed);
    ShowMessage("  SpeedOdoMaxRight ");
    ShowMessageln(SpeedOdoMaxRight);

    ShowMessage("OdoStartBrakeLeft ");
    ShowMessage(OdoStartBrakeLeft);
    ShowMessage("Ticks OdoStartBrakeRight ");
    ShowMessageln(OdoStartBrakeRight);
    ShowMessage("MaxOdoStateDuration ");
    ShowMessage(MaxOdoStateDuration);
    ShowMessageln(" ms");
  */
}



void Robot::motorControlOdo() {
  // call to reach a ODO cible on left AND right wheel so they don't stop at the same time accel and slow are used to smooth the movement of the mower
  //Stop motor independently when the cible is reach
  //
  if (UseBrakeLeft && (motorLeftSpeedRpmSet >= 0) && (stateEndOdometryLeft - odometryLeft <= -10)) {//Forward left need -10 because when stop the ticks can move in+ or- so do not stop before
    moveLeftFinish = true;
    PwmLeftSpeed = 0;
    motorLeftSpeedRpmSet = 0;
    motorLeftRpmCurr = 0;
  }
  if (UseBrakeRight && (motorRightSpeedRpmSet >= 0) && (stateEndOdometryRight - odometryRight <= -10)) {//right
    moveRightFinish = true;
    PwmRightSpeed = 0;
    motorRightSpeedRpmSet = 0;
    motorRightRpmCurr = 0;
  }
  //Reverse
  if (UseBrakeRight && (motorRightSpeedRpmSet <= 0) && (stateEndOdometryRight - odometryRight >= 10)) {//right
    moveRightFinish = true;
    PwmRightSpeed = 0;
    motorRightSpeedRpmSet = 0;
    motorRightRpmCurr = 0;
  }
  if (UseBrakeLeft && (motorLeftSpeedRpmSet <= 0) && (stateEndOdometryLeft - odometryLeft >= 10)) {//left
    moveLeftFinish = true;
    PwmLeftSpeed = 0;
    motorLeftSpeedRpmSet = 0;
    motorLeftRpmCurr = 0;
  }
  if (millis() < nextTimeMotorOdoControl) return;
  nextTimeMotorOdoControl = millis() + 15;

  //LEFT WHEEL
  leftSpeed = PwmLeftSpeed ; //Set first to Normal speed and stay like this if not change  by accel or brake so limit the compute time
  if (motorLeftSpeedRpmSet > 0) { //forward left wheel --------------------------------------------------------------------------
    if ((UseAccelLeft) && (millis() - stateStartTime < accelDurationLeft)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationLeft, 0, 89);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp)); //convert degree to radians
    }
    if (UseBrakeLeft && (odometryLeft > stateEndOdometryLeft - (OdoStartBrakeLeft))) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryLeft - odometryLeft), OdoStartBrakeLeft, 0, 89, 10);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp));
    }
    if (leftSpeed > SpeedOdoMaxLeft) leftSpeed = SpeedOdoMaxLeft;
    if (leftSpeed < SpeedOdoMin) leftSpeed = SpeedOdoMin; //Minimum speed to be sure the mower is always moving before stop
  }

  if (motorLeftSpeedRpmSet < 0) { //reverse left wheel ----------------------------------------------------------------------------
    if ((UseAccelLeft) && (millis() - stateStartTime < accelDurationLeft)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationLeft, 0, 89);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp)); //convert degree to radians
    }
    if (UseBrakeLeft && (odometryLeft < stateEndOdometryLeft + OdoStartBrakeLeft)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryLeft - odometryLeft), OdoStartBrakeLeft, 0, 89, 10);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp));
    }
    if (leftSpeed < SpeedOdoMaxLeft) leftSpeed = SpeedOdoMaxLeft;
    if (abs(leftSpeed) < SpeedOdoMin) leftSpeed = -SpeedOdoMin;
  }

  //  RIGHT WHEEL
  rightSpeed = PwmRightSpeed ; //Normal speed

  if (motorRightSpeedRpmSet > 0) { //forward Right wheel -----------------------------------------------------------------------------
    // ShowMessage(" FR rotate ");
    if (UseAccelRight && (millis() - stateStartTime < accelDurationRight)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationRight, 0, 89);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (UseBrakeRight && (odometryRight > stateEndOdometryRight - OdoStartBrakeRight)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryRight - odometryRight), OdoStartBrakeRight, 0, 89, 10);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (rightSpeed > SpeedOdoMaxRight) rightSpeed = SpeedOdoMaxRight;
    if (rightSpeed < SpeedOdoMin) rightSpeed = SpeedOdoMin;
  }
  if (motorRightSpeedRpmSet < 0) { //reverse Right wheel ------------------------------------------------------------------------------
    if (UseAccelRight && (millis() - stateStartTime < accelDurationRight)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationRight, 0, 89);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (UseBrakeRight && (odometryRight < stateEndOdometryRight +  OdoStartBrakeRight)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryRight - odometryRight), OdoStartBrakeRight, 0, 89, 10);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }

    if (rightSpeed < SpeedOdoMaxRight) rightSpeed = SpeedOdoMaxRight;
    if (abs(rightSpeed) < SpeedOdoMin) rightSpeed = -SpeedOdoMin;
  }

  //DRIVE IN STRAIGHT LINE
  if (stateCurr == STATE_FORWARD_ODO || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_DRIVE1_TO_NEWAREA) || (stateCurr == STATE_DRIVE2_TO_NEWAREA))  { //PID compute to accel or brake the wheel to drive straight
    motorRightPID.Kp = motorLeftPID.Kp;
    motorRightPID.Ki = motorLeftPID.Ki;
    motorRightPID.Kd = motorLeftPID.Kd;
    // USE THE IMU
    if ((imuUse) && (mowPatternCurr == MOW_LANES) && (stateCurr == STATE_FORWARD_ODO)) { //if mow by lane need different cible
      YawActualDeg = imu.ypr.yaw / PI * 180;
      if (laneUseNr == 1) {   //from -45 to 45 deg
        yawCiblePos = yawSet1 ;
        // ImuPidCiblePos= yawSet1+360;
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane1RollRight;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane1RollRight);
        } else {
          yawCibleNeg = yawOppositeLane1RollLeft;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane1RollLeft);
        }
      }
      if (laneUseNr == 2) {   //from 45 to 135 deg
        yawCiblePos = yawSet2;
        //ImuPidCiblePos= yawSet2;
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane2RollRight;
          // ImuPidCibleNeg = abs(yawOppositeLane2RollRight);
        } else {
          yawCibleNeg = yawOppositeLane2RollLeft;
          // ImuPidCibleNeg = abs(yawOppositeLane2RollLeft);
        }
      }
      if (laneUseNr == 3) {    //from 135 to -135 or 225 deg
        yawCiblePos = yawSet3;
        // ImuPidCiblePos= imu.rotate360(yawSet3);
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane3RollRight;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane3RollRight);
        } else {
          yawCibleNeg = yawOppositeLane3RollLeft;
          //  ImuPidCibleNeg = imu.rotate360(yawOppositeLane3RollLeft);
        }
      }

      if ((imu.ypr.yaw / PI * 180) > 0 ) imuDriveHeading = yawCiblePos;
      else imuDriveHeading = yawCibleNeg;
      imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
      imuDirPID.w = 0;
      imuDirPID.y_min = -motorSpeedMaxPwm / 2;
      imuDirPID.y_max = motorSpeedMaxPwm / 2;
      imuDirPID.max_output = motorSpeedMaxPwm / 2;
      imuDirPID.compute();
      /*
            if ((millis() - stateStartTime) < 1000) { // do not use rpm adjust during acceleration
              //bber402

              rightSpeed =  rightSpeed - (66 - (millis() - stateStartTime) / 30);
              leftSpeed =  leftSpeed - (66 - (millis() - stateStartTime) / 30);
              if (rightSpeed < 0 ) rightSpeed = 0;
              if (leftSpeed < 0 ) leftSpeed = 0;
            }
            else //adjust rpm speed only after 1 seconde
            {
      */
      //bber400 //adjust RPM speed
      //PID version
      motorRightPID.x = motorRightRpmCurr; //16/10/22
      //motorRightPID.w = motorSpeedMaxRpm;
      motorRightPID.w = motorRightSpeedRpmSet;
      motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
      motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
      motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
      motorRightPID.compute();
      //ShowMessageln(motorRightPID.y);
      motorRpmCoeff = (100 + motorRightPID.y) / 100;
      if (motorRpmCoeff < 0.10) motorRpmCoeff = 0.10;
      if (motorRpmCoeff > 2.00) motorRpmCoeff = 2.00;
/*
 * Speed control loop to build !!!
      if ((motorRightSpeedRpmSet / motorRightRpmCurr) < 0.8 ) { //speed real is 20 % too high need a brake
        Serial.print("R speed  ");
        Serial.print((motorRightSpeedRpmSet / motorRightRpmCurr));
        Serial.print(" / ");
        Serial.println(motorRightPID.y);
      }

      if ((motorLeftSpeedRpmSet / motorLeftRpmCurr) < 0.8 ) {
        Serial.print("L speed  ");
        Serial.print((motorLeftSpeedRpmSet / motorLeftRpmCurr));
        Serial.print(" / ");
        Serial.println(motorRightPID.y);
      }
*/
      if ((sonarSpeedCoeff != 1) || (!autoAdjustSlopeSpeed)) { //do not change speed if sonar is activate
        motorRpmCoeff = 1;
      }
      if (highGrassDetect) motorRpmCoeff = highGrassSpeedCoeff; //reduce speed when mower detect high grass
      rightSpeed = motorRpmCoeff * (rightSpeed + imuDirPID.y / 2);
      leftSpeed =  motorRpmCoeff * (leftSpeed - imuDirPID.y / 2);
    } else { //// NORMAL MOWING OR PERIFIND
      if (imuUse) { /// use the IMU for straight line
        YawActualDeg = imu.ypr.yaw / PI * 180;
        // if(abs(YawActualDeg) >90) YawMedianDeg = imu.rotate360(YawActualDeg);
        // else YawMedianDeg= YawActualDeg+90;
        imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
        imuDirPID.w = 0;
        imuDirPID.y_min = -motorSpeedMaxPwm / 2;
        imuDirPID.y_max = motorSpeedMaxPwm / 2;
        imuDirPID.max_output = motorSpeedMaxPwm / 2;
        imuDirPID.compute();
        //bber400 //adjust RPM speed
        //PID version
        motorRightPID.x = motorRightRpmCurr;
        //motorRightPID.w = motorSpeedMaxRpm;
        motorRightPID.w = motorRightSpeedRpmSet;//16/10/22
        motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
        motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
        motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
        motorRightPID.compute();
        //ShowMessageln(motorRightPID.y);
        motorRpmCoeff = (100 + motorRightPID.y) / 100;
        if (motorRpmCoeff < 0.10) motorRpmCoeff = 0.10;
        if (motorRpmCoeff > 2.00) motorRpmCoeff = 2.00;

        if ((sonarSpeedCoeff != 1) || (!autoAdjustSlopeSpeed)) { //do not change speed if sonar is activate
          motorRpmCoeff = 1;
        }
        rightSpeed = motorRpmCoeff * (rightSpeed + imuDirPID.y / 2);
        leftSpeed =  motorRpmCoeff * (leftSpeed - imuDirPID.y / 2);
      } else { 
        /// use only the odometry  for straight line
        if (millis() >= nextTimePidCompute) { //to go in straight line need to compute only each 200 milliseconde and add the dif to one wheel
          nextTimePidCompute = millis() + 200;
          //stateStartOdometryLeft = stateStartOdometryLeft + ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft)); // very important change the odo to retrieve the line to avoid drift
          motorRightPID.x = ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft));
          motorRightPID.w = 0;
          motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
          motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
          motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
          motorRightPID.compute();
          rightSpeed =  rightSpeed + motorRightPID.y / 2;
          leftSpeed =  leftSpeed - motorRightPID.y / 2;
        }
      }
    }

    //bber200
    //bber200 reduce perimeter speed only if both perimeter and sonar are actif
    if (perimeterSpeedCoeff == 1) {
      rightSpeed = rightSpeed * sonarSpeedCoeff;
      leftSpeed = leftSpeed * sonarSpeedCoeff;
    } else {
      rightSpeed = rightSpeed * perimeterSpeedCoeff;
      leftSpeed = leftSpeed * perimeterSpeedCoeff;
    }

    if (rightSpeed > 255) rightSpeed = 255;
    if (leftSpeed > 255) leftSpeed = 255;
    if (rightSpeed < 0) rightSpeed = 0;
    if (leftSpeed < 0) leftSpeed = 0;
  }

  if (stateCurr != STATE_OFF) {
    /*
      if (perimeterSpeedCoeff != 1) {
      ShowMessageln(perimeterSpeedCoeff);
      }

        ShowMessage(millis());
        ShowMessage(" Moving Average Dist= ");
        ShowMessage(currDistToDrive);
        ShowMessage(" ODO **** Lspeed= ");
        ShowMessage(leftSpeed);
        ShowMessage(" ODO Start/Actual/End ");
        ShowMessage(stateStartOdometryLeft);
        ShowMessage("/");
        ShowMessage(odometryLeft);
        ShowMessage("/");
        ShowMessage(stateEndOdometryLeft);
        ShowMessage(" ************************* Rspeed= ");
        ShowMessage(rightSpeed);
        ShowMessage(" ODO Start/Actual/End ");
        ShowMessage(stateStartOdometryRight);
        ShowMessage("/");
        ShowMessage(odometryRight);
        ShowMessage("/");
        ShowMessage(stateEndOdometryRight);
        ShowMessage(" PID reel ");
        ShowMessage(motorRightPID.x);
        ShowMessage(" PID resultat du calcul ");
        ShowMessageln(motorRightPID.y);
        ShowMessage("IMU ***** Line use ");
        ShowMessage(laneUseNr);
        ShowMessage(" imuDriveHeading ");
        ShowMessage(imuDriveHeading);
        ShowMessage(" YawMedianDeg ");
        ShowMessage(YawMedianDeg);
        ShowMessage(" YawActualDeg ");
        ShowMessage(YawActualDeg);
        ShowMessage(" correctRight ");
        ShowMessage(correctRight);
        ShowMessage(" correctLeft ");
        ShowMessage(correctLeft);
        ShowMessage(" PID reel ");
        ShowMessage(imuDirPID.x);
        ShowMessage(" PID resultat du calcul ");
        ShowMessageln(imuDirPID.y);
        ShowMessage(" imu.ypr.yaw ");
        ShowMessageln(imu.ypr.yaw);
    */
  }
  setMotorPWM(leftSpeed, rightSpeed);
}



// PID controller: track perimeter
void Robot::motorControlPerimeter() {
  if (millis() < nextTimeMotorPerimeterControl) return;
  nextTimeMotorPerimeterControl = millis() + 15; //bb read the perimeter each 15 ms
  //never stop the PID compute while turning for the new transition
  //use the perimeterMagLeft as cible to smooth the tracking
  //Value reference perimeterMagMaxValue , maybe need to be calculate in mower setting up procedure
  perimeterPID.x = 5 * (double(perimeterMagLeft) / perimeterMagMaxValue);
  if (perimeterInsideLeft)  perimeterPID.w = -0.5;
  else     perimeterPID.w = 0.5;
  perimeterPID.y_min = -ActualSpeedPeriPWM ;
  perimeterPID.y_max = ActualSpeedPeriPWM ;
  perimeterPID.max_output = ActualSpeedPeriPWM ;
  perimeterPID.compute();

  if ((millis() > stateStartTime + 10000) && (millis() > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut)) {
    // robot is wheel-spinning while tracking => roll to get ground again
    if (trakBlockInnerWheel == 0) {
      if (perimeterInsideLeft) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = -ActualSpeedPeriPWM / 2;
      } else {
        rightSpeedperi = -ActualSpeedPeriPWM / 2;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (trakBlockInnerWheel == 1) {
      if (perimeterInsideLeft) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = 0;
      } else {
        rightSpeedperi = 0;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SEARCH;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagLeft);
      ShowMessage(";");
      ShowMessage(perimeterInsideLeft);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
    if (track_ClockWise) {
      setMotorPWM( leftSpeedperi, rightSpeedperi);
    } else {
      setMotorPWM( rightSpeedperi, leftSpeedperi);
    }
    lastTimeForgetWire = millis();

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut) {
      if (perimeterInsideLeft) {
        ShowMessageln("Tracking Fail and we are inside, So start to find again the perimeter");
        periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      } else {
        ShowMessageln("Tracking Fail and we are outside, So start to roll to find again the perimeter");
        if (track_ClockWise) {
          rollDir = 0; //le 02/04/22 need to check if it's the correct roll dir
        } else {
          rollDir = 1;
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
      }
    }
    return;
  }
  if ((millis() - lastTimeForgetWire ) < trackingPerimeterTransitionTimeOut) {
    //PeriCoeffAccel move gently from 3 to 1 and so perimeterPID.y/PeriCoeffAccel increase during 3 secondes
    PeriCoeffAccel = (3000.00 - (millis() - lastTimeForgetWire)) / 1000.00 ;
    if (PeriCoeffAccel < 1.00) PeriCoeffAccel = 1.00;
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 +  perimeterPID.y / PeriCoeffAccel));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 -  perimeterPID.y / PeriCoeffAccel));
    //bber30 we are in sonartrigger ,so maybe near station , so avoid 1 wheel reverse because station check is forward
    if (ActualSpeedPeriPWM != MaxSpeedperiPwm) {
      if (rightSpeedperi < 0) rightSpeedperi = 0;
      if (leftSpeedperi < 0) leftSpeedperi = 0;
    }

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SLOW;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagLeft);
      ShowMessage(";");
      ShowMessage(perimeterInsideLeft);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  } else {
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5   + perimeterPID.y));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  - perimeterPID.y));

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("FAST;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagLeft);
      ShowMessage(";");
      ShowMessage(perimeterInsideLeft);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }

  //bb2
  if ((millis() - stateStartTime ) < 2000) { //at the start of the tracking accelerate slowly during 2 secondes
    //leftSpeedperi = leftSpeedperi - (66 - (millis() - stateStartTime) / 30);
    leftSpeedperi = int(leftSpeedperi * ((millis() - stateStartTime) / 2000));
    //bber300
    if (leftSpeedperi < SpeedOdoMin) leftSpeedperi = SpeedOdoMin;
    //rightSpeedperi = rightSpeedperi - (66 - (millis() - stateStartTime) / 30);
    rightSpeedperi = int(rightSpeedperi * ((millis() - stateStartTime) / 2000));
    if (rightSpeedperi < SpeedOdoMin) rightSpeedperi = SpeedOdoMin;
  }

  if (track_ClockWise) {
    setMotorPWM( leftSpeedperi, rightSpeedperi);
  } else {
    setMotorPWM( rightSpeedperi, leftSpeedperi);
  }
  if (abs(perimeterMagLeft) < perimeterMagMaxValue / 4) { //250 can be replace by timedOutIfBelowSmag to be tested
    perimeterLastTransitionTime = millis(); //initialise perimeterLastTransitionTime if perfect sthraith line
  }
}



void Robot::motorControlPerimeter2Coil() {
  if (millis() < nextTimeMotorPerimeterControl) return;
  nextTimeMotorPerimeterControl = millis() + 15; //bb read the perimeter each 15 ms
  //never stop the PID compute while turning for the new transition
  //use the PerimeterMagRight as cible to smooth the tracking
  //Value reference perimeterMagMaxValue , maybe need to be calculate in mower setting up procedure
  perimeterPID.x = 5 * (double(perimeterMagRight) / perimeterMagMaxValue);
  if (perimeterInsideRight)  perimeterPID.w = -0.5;
  else     perimeterPID.w = 0.5;

  perimeterPID.y_min = -ActualSpeedPeriPWM ;
  perimeterPID.y_max = ActualSpeedPeriPWM ;
  perimeterPID.max_output = ActualSpeedPeriPWM ;
  perimeterPID.compute();

  if (!(perimeterInsideLeft) || ((millis() > stateStartTime + 10000) && (millis() > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut))) {
    // robot is wheel-spinning while tracking => roll to get ground again
    if (trakBlockInnerWheel == 0) {
      if (perimeterInsideRight) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = -ActualSpeedPeriPWM / 2;
      } else {
        rightSpeedperi = -ActualSpeedPeriPWM / 2;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (trakBlockInnerWheel == 1) {
      if (perimeterInsideRight) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = 0;
      } else {
        rightSpeedperi = 0;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SEARCH;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagRight);
      ShowMessage(";");
      ShowMessage(perimeterInsideRight);
      ShowMessage(";   ");
      ShowMessage (perimeterPID.x);
      ShowMessage("    ;");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
    if (track_ClockWise) {
      setMotorPWM( leftSpeedperi, rightSpeedperi);
    } else {
      setMotorPWM( rightSpeedperi, leftSpeedperi);
    }
    lastTimeForgetWire = millis();

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut) {
      if (perimeterInsideRight) {
        ShowMessageln("Tracking Fail and we are inside, So start to find again the perimeter");
        periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      } else {
        ShowMessageln("Tracking Fail and we are outside, So start to roll to find again the perimeter");
        if (track_ClockWise) {
          rollDir = 0; //le 02/04/22 need to check if it's the correct roll dir
        } else {
          rollDir = 1;
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
      }
    }
    return;
  }
  if ((millis() - lastTimeForgetWire ) < trackingPerimeterTransitionTimeOut) {
    //PeriCoeffAccel move gently from 3 to 1 and so perimeterPID.y/PeriCoeffAccel increase during 3 secondes
    PeriCoeffAccel = (3000.00 - (millis() - lastTimeForgetWire)) / 1000.00 ;
    if (PeriCoeffAccel < 1.00) PeriCoeffAccel = 1.00;
    rightSpeedperi = max(0,  ActualSpeedPeriPWM  + (perimeterPID.y ) / PeriCoeffAccel);
    leftSpeedperi = max(0,  ActualSpeedPeriPWM  - (perimeterPID.y ) / PeriCoeffAccel);
    //bber30 we are in sonartrigger ,so maybe near station , so avoid 1 wheel reverse because station check is forward
    if (ActualSpeedPeriPWM != MaxSpeedperiPwm) {
      if (rightSpeedperi < 0) rightSpeedperi = 0;
      if (leftSpeedperi < 0) leftSpeedperi = 0;
    }

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("SLOW;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagRight);
      ShowMessage(";");
      ShowMessage(perimeterInsideRight);
      ShowMessage(";     ");
      ShowMessage (perimeterPID.x);
      ShowMessage("     ;     ");
      ShowMessage(perimeterPID.y);
      ShowMessage("   ;");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  } else {
    rightSpeedperi = max(0, ActualSpeedPeriPWM +  perimeterPID.y);
    leftSpeedperi = max(0, ActualSpeedPeriPWM - perimeterPID.y);

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("FAST;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMagRight);
      ShowMessage(";");
      ShowMessage(perimeterInsideRight);
      ShowMessage(";    ");
      ShowMessage (perimeterPID.x);
      ShowMessage("   ;  ");
      ShowMessage(perimeterPID.y);
      ShowMessage("   ;");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }
  //bb2
  if ((millis() - stateStartTime ) < 2000) { //at the start of the tracking accelerate slowly during 2 secondes
    //leftSpeedperi = leftSpeedperi - (66 - (millis() - stateStartTime) / 30);
    leftSpeedperi = int(leftSpeedperi * ((millis() - stateStartTime) / 2000));
    //bber300
    if (leftSpeedperi < SpeedOdoMin) leftSpeedperi = SpeedOdoMin;
    //rightSpeedperi = rightSpeedperi - (66 - (millis() - stateStartTime) / 30);
    rightSpeedperi = int(rightSpeedperi * ((millis() - stateStartTime) / 2000));
    if (rightSpeedperi < SpeedOdoMin) rightSpeedperi = SpeedOdoMin;
  }

  if (track_ClockWise) {
    setMotorPWM( leftSpeedperi, rightSpeedperi);
  } else {
    setMotorPWM( rightSpeedperi, leftSpeedperi);
  }

  if (abs(perimeterMagRight) < perimeterMagMaxValue / 4) { //250 can be replace by timedOutIfBelowSmag to be tested
    perimeterLastTransitionTime = millis(); //initialise perimeterLastTransitionTime if perfect sthraith line
  }
}



// check for odometry sensor faults
void Robot::checkOdometryFaults() {
  // if pwm > 1/3 of maxvalue the rpm need to be !=0 or it's error
  boolean leftErr = false;
  boolean rightErr = false;
  if ((stateCurr == STATE_FORWARD_ODO) &&  (millis() - stateStartTime > 8000) ) {
    // just check if odometry sensors may not be working at all
    if ( (motorLeftPWMCurr > motorSpeedMaxPwm / 3) && (abs(motorLeftRpmCurr) < 1)  )  leftErr = true;
    if ( (motorRightPWMCurr > motorSpeedMaxPwm / 3) && (abs(motorRightRpmCurr) < 1)  ) rightErr = true;
  }

  if (leftErr) {
    ShowMessage("Left odometry error: PWM=");
    ShowMessage(motorLeftPWMCurr);
    ShowMessage("\tRPM=");
    ShowMessageln(motorLeftRpmCurr);
    addErrorCounter(ERR_ODOMETRY_LEFT);
    setNextState(STATE_ERROR, 0);
  }

  if (rightErr) {
    ShowMessage("Right odometry error: PWM=");
    ShowMessage(motorRightPWMCurr);
    ShowMessage("\tRPM=");
    ShowMessageln(motorRightRpmCurr);
    addErrorCounter(ERR_ODOMETRY_RIGHT);
    setNextState(STATE_ERROR, 0);
  }
}



void Robot::motorControl() {
  if (millis() < nextTimeMotorControl) return;
  nextTimeMotorControl = millis() + 100;  // 10 at the original
  //static unsigned long nextMotorControlOutputTime = 0;
  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen hÃ¶chstes Drehmoment fÃ¼r die Solldrehzahl zu gewÃ¤hrleisten
  motorLeftPID.w = motorLeftSpeedRpmSet;               // SOLL
  motorRightPID.w = motorRightSpeedRpmSet;             // SOLL

  float RLdiff = motorLeftRpmCurr - motorRightRpmCurr;

  if (motorLeftSpeedRpmSet == motorRightSpeedRpmSet) {
    // line motion
    if (odoLeftRightCorrection) {
      motorLeftPID.w = motorLeftSpeedRpmSet - RLdiff / 2;
      motorRightPID.w = motorRightSpeedRpmSet + RLdiff / 2;
    }
  }
  motorLeftPID.x = motorLeftRpmCurr;                 // IST
  if ((stateCurr == STATE_OFF)) motorLeftPID.w = 0; // to be sure the motor stop when OFF
  /*
    motorLeftPID.y_min = -motorSpeedMaxPwm;        // Regel-MIN
    motorLeftPID.y_max = motorSpeedMaxPwm;     // Regel-MAX
    motorLeftPID.max_output = motorSpeedMaxPwm;    // Begrenzung
    motorLeftPID.compute();
    leftSpeed = int(motorLeftPWMCurr + motorLeftPID.y);
    if (motorLeftSpeedRpmSet > 0) leftSpeed = min( max(0, leftSpeed), motorSpeedMaxPwm);
    if (motorLeftSpeedRpmSet < 0) leftSpeed = max(-motorSpeedMaxPwm, min(0, leftSpeed));
  */
  motorLeftPID.y_min = -255;        // Regel-MIN
  motorLeftPID.y_max = 255;     // Regel-MAX
  motorLeftPID.max_output = 255;    // Begrenzung
  motorLeftPID.compute();
  leftSpeed = int(motorLeftPWMCurr + motorLeftPID.y);
  if (motorLeftSpeedRpmSet > 0) leftSpeed = min( max(0, leftSpeed), 255);
  if (motorLeftSpeedRpmSet < 0) leftSpeed = max(-255, min(0, leftSpeed));
  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen hÃ¶chstes Drehmoment fÃ¼r die Solldrehzahl zu gewÃ¤hrleisten
  motorRightPID.Kp = motorLeftPID.Kp;
  motorRightPID.Ki = motorLeftPID.Ki;
  motorRightPID.Kd = motorLeftPID.Kd;
  motorRightPID.x = motorRightRpmCurr;               // IST
  if ((stateCurr == STATE_OFF)) motorRightPID.w = 0; // to be sure the motor stop when OFF
  /*
    motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
    motorRightPID.y_max = motorSpeedMaxPwm;        // Regel-MAX
    motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
    motorRightPID.compute();
    rightSpeed = int(motorRightPWMCurr + motorRightPID.y);
    if (motorRightSpeedRpmSet > 0) rightSpeed = min( max(0, rightSpeed), motorSpeedMaxPwm);
    if (motorRightSpeedRpmSet < 0) rightSpeed = max(-motorSpeedMaxPwm, min(0, rightSpeed));
  */
  motorRightPID.y_min = -255;       // Regel-MIN
  motorRightPID.y_max = 255;        // Regel-MAX
  motorRightPID.max_output = 255;   // Begrenzung
  motorRightPID.compute();
  rightSpeed = int(motorRightPWMCurr + motorRightPID.y);
  if (motorRightSpeedRpmSet > 0) rightSpeed = min( max(0, rightSpeed), 255);
  if (motorRightSpeedRpmSet < 0) rightSpeed = max(-255, min(0, rightSpeed));

  if ( (abs(motorLeftPID.x) < 2) && (abs(motorLeftPID.w) < 0.1) ) leftSpeed = 0; // ensures PWM is really zero
  if ( (abs(motorRightPID.x)  < 2) && (abs(motorRightPID.w) < 0.1) ) rightSpeed = 0; // ensures PWM is really zero
  /*  if (millis() >= nextMotorControlOutputTime){
      nextMotorControlOutputTime = millis() + 200;
      ShowMessage("PID x=");
      ShowMessage(motorLeftPID.x);
      ShowMessage("\tPID w=");
      ShowMessage(motorLeftPID.w);
      ShowMessage("\tPID y=");
      ShowMessage(motorLeftPID.y);
      ShowMessage("\tPWM=");
      ShowMessageln(leftSpeed);
    }
  */
  setMotorPWM(leftSpeed, rightSpeed);
}



void Robot::motorMowControl() {
  if (millis() < nextTimeMotorMowControl) return;
  nextTimeMotorMowControl = millis() + 100;
  if (motorMowForceOff) motorMowEnable = false;
  //Auto adjust the motor speed according to cutting power (The goal is On high grass the motor rotate faster)
  //A runningmedian process is used to check each seconde the power value of mow motor
  //if power is low the speed is reduce to have a longer mowing duration and less noise.
  if (motorMowEnable) {
    motorMowPowerMedian.add(motorMowPower);
    if (motorMowPowerMedian.getCount() > 10) { //check each 1 secondes
      int prevcoeff =  motorMowPwmCoeff;
      motorMowPwmCoeff = int((100 * motorMowPowerMedian.getAverage(4)) / (0.8 * motorMowPowerMax));
      if (motorMowPwmCoeff < prevcoeff) {
        //filter on speed reduce to keep the mow speed high for longuer duration
        motorMowPwmCoeff = int((0.1) * motorMowPwmCoeff + (0.9) * prevcoeff);// use only 10% of the new value
      }

      if (motorMowPwmCoeff > 100) motorMowPwmCoeff = 100;
      if (motorMowEnable) {
        motorMowSpeedPWMSet = motorMowSpeedMinPwm + ((double)(motorMowSpeedMaxPwm - motorMowSpeedMinPwm)) * (((double)motorMowPwmCoeff) / 100.0);
      }
      if (motorMowSpeedPWMSet < motorMowSpeedMinPwm) motorMowSpeedPWMSet = motorMowSpeedMinPwm;
      if (motorMowSpeedPWMSet > motorMowSpeedMaxPwm) motorMowSpeedPWMSet = motorMowSpeedMaxPwm;
      //max speed on wire and spirale
      motorMowPowerMedian.clear();
    }
  } else {
    motorMowSpeedPWMSet = 0;
  }
  if (stateCurr == STATE_ERROR) {
    setMotorMowPWM(0, false); //stop immediatly on error (tilt etc....)
  } else {
    setMotorMowPWM(motorMowSpeedPWMSet, true);
  }
}



// calculate map position by odometry sensors
void Robot::calcOdometry() {
  if ((millis() < nextTimeOdometry) || (stateCurr == STATE_OFF)) return;
  nextTimeOdometry = millis() + 100; //bb 300 at the original but test less
  static int lastOdoLeft = 0;
  static int lastOdoRight = 0;
  int odoLeft = odometryLeft;
  int odoRight = odometryRight;
  int ticksLeft = odoLeft - lastOdoLeft;
  int ticksRight = odoRight - lastOdoRight;
  lastOdoLeft = odoLeft;
  lastOdoRight = odoRight;
  double left_cm = ((double)ticksLeft) / ((double)odometryTicksPerCm);
  double right_cm = ((double)ticksRight) / ((double)odometryTicksPerCm);
  double avg_cm  = (left_cm + right_cm) / 2.0;
  double wheel_theta = (left_cm - right_cm) / ((double)odometryWheelBaseCm);
  //odometryTheta += wheel_theta;
  odometryTheta = scalePI(odometryTheta - wheel_theta);
  motorLeftRpmCurr  = double ((( ((double)ticksLeft) / ((double)odometryTicksPerRevolution)) / ((double)(millis() - lastMotorRpmTime))) * 60000.0);
  motorRightRpmCurr = double ((( ((double)ticksRight) / ((double)odometryTicksPerRevolution)) / ((double)(millis() - lastMotorRpmTime))) * 60000.0);
  lastMotorRpmTime = millis();
  if (stateCurr == STATE_PERI_TRACK)  totalDistDrive = totalDistDrive + int(avg_cm);
  currDistToDrive = currDistToDrive + int(avg_cm);
  if (imuUse) {
    odometryX += avg_cm * sin(prevYawCalcOdo);
    odometryY += avg_cm * cos(prevYawCalcOdo);
    //prevYawCalcOdo = imu.ypr.yaw;
  } else {
    // FIXME: theta should be old theta, not new theta?
    odometryX += avg_cm * sin(odometryTheta);
    odometryY += avg_cm * cos(odometryTheta);
  }
}



void Robot::testMotors() {
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;  
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  ShowMessageln(F("testing left motor (forward) half speed..."));
  delay(100);

  motorLeftPWMCurr = motorSpeedMaxPwm / 2; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);

  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  ShowMessageln(F("testing left motor (reverse) full speed..."));
  delay(100);

  motorLeftPWMCurr = -motorSpeedMaxPwm; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);

  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  ShowMessageln(F("testing right motor (forward) half speed..."));
  delay(100);

  motorLeftPWMCurr = 0; motorRightPWMCurr = motorSpeedMaxPwm / 2;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);

  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  ShowMessageln(F("testing right motor (reverse) full speed..."));
  delay(100);

  motorLeftPWMCurr = 0; motorRightPWMCurr = -motorSpeedMaxPwm;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
  delayInfo(5000);

  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr);
}



void Robot::OdoRightCountInt() {
  if (robot.motorRightPWMCurr >= 0 ) robot.odometryRight++; else robot.odometryRight--;
  asm("dsb");
}


void Robot::OdoLeftCountInt() {
  // Pulse01 = micros() - Pulse02;
  // if (Pulse01 > 10  ) { // debounce for 120uS
  //   Pulse02 = micros();
  if (robot.motorLeftPWMCurr > 0 ) robot.odometryLeft++; else robot.odometryLeft--;
  //}
  asm("dsb");
}



// check motor current
void Robot::checkCurrent() {
  if (millis() < nextTimeCheckCurrent) return;
  nextTimeCheckCurrent = millis() + 100;
  // *************MOW MOTOR***********************
  if ((statusCurr == NORMAL_MOWING) && (!highGrassDetect)) {  //do not start the spirale if in tracking and motor detect high grass
    if (motorMowPower >= 0.8 * motorMowPowerMax) {
      spiraleNbTurn = 0;

      highGrassDetect = true;
      ShowMessageln("Warning  motorMowPower >= 0.8 * motorMowPowerMax ");
      ////  http://forums.parallax.com/discussion/comment/1326585#Comment_1326585
    } else {
      if ((spiraleNbTurn >= 8)) {
        spiraleNbTurn = 0;

        highGrassDetect = false; //stop the spirale
      }
    }
  }
  // if (motorMowPower >= motorMowPowerMax)
  if ((motorMowEnable) && (motorMowPower >= motorMowPowerMax)) {
    motorMowSenseCounter++;
    ShowMessage("Warning  motorMowPower >= motorMowPowerMax : ");
    ShowMessageln(motorMowSenseCounter);
  } else {
    errorCounterMax[ERR_MOW_SENSE] = 0;
    motorMowSenseCounter = 0;
    if ((lastTimeMotorMowStuck != 0) && (millis() >= lastTimeMotorMowStuck + 60000)) { // wait 60 seconds before switching on again
      errorCounter[ERR_MOW_SENSE] = 0;
      if ((stateCurr == STATE_FORWARD_ODO)) { //avoid risq of restart not allowed
        motorMowEnable = true;
        lastTimeMotorMowStuck = 0;
        ShowMessageln("Time to restart the mow motor after the 60 secondes pause");
      }
    }
  }
  //need to check this
  if (motorMowSenseCounter >= 10) { //ignore motorMowPower for 1 seconds
    motorMowEnable = false;
    ShowMessageln("Motor mow power overload. Motor STOP and try to start again after 1 minute");
    addErrorCounter(ERR_MOW_SENSE);
    lastTimeMotorMowStuck = millis();
  }
  //**************drive motor***********************
  //bb add test current in manual mode and stop immediatly
  if (statusCurr == MANUAL) {
    if (motorLeftPower >= 0.8 * motorPowerMax) {
      ShowMessage("Motor Left power is 80 % of the max, value --> ");
      ShowMessageln(motorLeftPower);
      setMotorPWM(0, 0);
      setNextState(STATE_OFF, 0);
    }
    if (motorRightPower >= 0.8 * motorPowerMax) {
      ShowMessage("Motor Right power is 80 % of the max, value --> ");
      ShowMessageln(motorRightPower);
      setMotorPWM(0, 0);
      setNextState(STATE_OFF, 0);
    }
  }
  // here in auto mode
  if (millis() > stateStartTime + motorPowerIgnoreTime) {
    //Motor right****************************************************************
    //First react test to 80 % powerMax
    if (motorRightPower >= 0.8 * motorPowerMax) {
      motorRightSenseCounter++;
      setBeeper(500, 500, 0, 2000, 0);
      setMotorPWM(0, 0);
      ShowMessage("Motor Right power is 80 % of the max, value --> ");
      ShowMessageln(motorRightPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          ShowMessageln("Power motor left warning ");
          setNextState(STATE_STATION_CHECK, rollDir);
          return;
        } else {
          if (mowPatternCurr == MOW_LANES) reverseOrBidir(rollDir);
          else reverseOrBidir(LEFT);
        }
      }
    } else {
      setBeeper(0, 0, 0, 0, 0);
      motorRightSenseCounter = 0; // the sense is OK reset all the counter
    }
    //Second test at powerMax by increase the counter to stop to error
    if (motorRightPower >= motorPowerMax) {
      motorRightSenseCounter++;
      // setMotorPWM(0, 0);
      //addErrorCounter(ERR_MOTOR_RIGHT);
      //setNextState(STATE_ERROR, 0);
      ShowMessage("Warning: Motor Right power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      ShowMessageln(motorRightSenseCounter);
    }
    //Motor left****************************************************************
    //First react test to 80 % powerMax
    if (motorLeftPower >= 0.8 * motorPowerMax) {
      motorLeftSenseCounter++;
      setBeeper(1000, 1000, 0, 2000, 0);
      setMotorPWM(0, 0);
      ShowMessage("Motor Left power is 80 % of the max, value --> ");
      ShowMessageln(motorLeftPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          ShowMessageln("Power motor left warning ");
          setNextState(STATE_STATION_CHECK, rollDir);
          return;
        } else {
          if (mowPatternCurr == MOW_LANES) reverseOrBidir(rollDir);
          else reverseOrBidir(RIGHT);
        }
      }
    } else {
      setBeeper(0, 0, 0, 0, 0);
      motorLeftSenseCounter = 0; // the sense is OK reset the counter
    }
    //Second test at powerMax by increase the counter to stop to error
    if (motorLeftPower >= motorPowerMax) {
      motorLeftSenseCounter++;
      // setMotorPWM(0, 0);
      //addErrorCounter(ERR_MOTOR_LEFT);
      //setNextState(STATE_ERROR, 0);
      ShowMessage("Warning: Motor Left power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      ShowMessageln(motorLeftSenseCounter);
    }
    //final test on the counter to generate the error and stop the mower
    if (motorLeftSenseCounter >= 10) { //the motor is stuck for more than 1 seconde 10 * 100 ms go to error.
      ShowMessage("Fatal Error: Motor Left power over 100% for more than 1 seconde last power --> ");
      ShowMessageln(motorLeftPower);
      addErrorCounter(ERR_MOTOR_LEFT);
      setMotorPWM(0, 0);
      setNextState(STATE_ERROR, 0);
    }
    if (motorRightSenseCounter >= 10) { //the motor is stuck for more than 1 seconde go to error.
      ShowMessage("Fatal Error: Motor Right power over 100% for more than 1 seconde last power --> ");
      ShowMessageln(motorRightPower);
      addErrorCounter(ERR_MOTOR_RIGHT);
      setMotorPWM(0, 0);
      setNextState(STATE_ERROR, 0);
    }
  } //motorpower ignore time
}