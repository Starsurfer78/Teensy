// check bumpers
void Robot::checkBumpers() {
  if ((millis() < 3000) || (!bumperUse)) return;
  /*
    if (stateCurr=STATE_PERI_OUT_ROLL_TOINSIDE){
    if (bumperLeft) {
      rollDir=RIGHT;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
       setMotorPWM(0, 0);

      return;
    }
    if (bumperRight){
      rollDir=LEFT;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
       setMotorPWM(0, 0);

      return;
    }
    }
  */
  if ((bumperLeft || bumperRight || bumperRearLeft || bumperRearRight)) {
    if (statusCurr == MANUAL) {
      ShowMessageln("Bumper trigger in Manual mode ?????????");
      setNextState(STATE_OFF, 0); //the bumper stop all in manual mode
    } else {
      spiraleNbTurn = 0;
      highGrassDetect = false;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
      setMotorPWM(0, 0);
      if (bumperLeft || bumperRearLeft) {
        ShowMessageln("Bumper left trigger");
        reverseOrBidir(LEFT);
      } else {
        ShowMessageln("Bumper right trigger");
        reverseOrBidir(RIGHT);
      }
    }
  }
}



// check bumpers while tracking perimeter
void Robot::checkBumpersPerimeter() {
  if ((bumperLeft || bumperRight || bumperRearLeft || bumperRearRight)) { // the bumper is used to detect the station
    motorLeftRpmCurr = motorRightRpmCurr = 0 ;
    setMotorPWM(0, 0);//stop immediatly and station check to see if voltage on pin
    ShowMessageln("Bump on Something check for station");
    setNextState(STATE_STATION_CHECK, rollDir);
    return;
  }
  //if (!UseBumperDock) {   // read the station voltage
  //bber300
  if ((powerboard_I2c_line_Ok) && (millis() >= nextTimeReadStationVoltage)) {
    nextTimeReadStationVoltage = millis() + 20;
    chgVoltage = ChargeIna226.readBusVoltage() ;
  }
  if (chgVoltage > 5) {
    motorLeftRpmCurr = motorRightRpmCurr = 0 ;
    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
    setMotorPWM(0, 0);//stop immediatly and wait 2 sec to see if voltage on pin
    ShowMessageln("Detect a voltage on charging contact");
    setNextState(STATE_STATION_CHECK, rollDir);
    return;
  }
  //}
}