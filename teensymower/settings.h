void Robot::loadSaveRobotStats(boolean readflag) {
  int addr = ADDR_ROBOT_STATS;
  //create a new history file name to separate the data log on sd card
  // sd.open need a char array to work
  sprintf(historyFilenameChar, "%02d%02d%02d%02d%02d.txt", datetime.date.year - 2000, datetime.date.month, datetime.date.day, datetime.time.hour, datetime.time.minute);

  if (sdCardReady) {
    ShowMessage(F("Log Filename : "));
    ShowMessageln(historyFilenameChar);
  }
  if (readflag) {
    ShowMessage(F("Load Stats "));
  } else {
    ShowMessage(F("Save Stats "));
  }

  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    ShowMessageln(F("********************************************"));
    ShowMessageln(F("PLEASE CHECK IF YOUR ROBOT STATS ARE CORRECT"));
    ShowMessageln(F("********************************************"));
  }
  eereadwrite(readflag, addr, statsMowTimeMinutesTrip);
  eereadwrite(readflag, addr, statsMowTimeMinutesTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCounterTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTrip);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityAverage);
  // <----------------------------new robot stats to save goes here!----------------
  ShowMessage(F("Adress Start = "));
  ShowMessage(ADDR_ROBOT_STATS);
  ShowMessage(F(" Stop = "));
  ShowMessageln(addr);
}

void Robot::loadSaveUserSettings(boolean readflag) {
  int addr = ADDR_USER_SETTINGS;
  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic

  if ((readflag) && (magic != MAGIC)) {
    ShowMessageln(F("************************************"));
    ShowMessageln(F("        NO EEPROM USER DATA"));
    ShowMessageln(F("PLEASE CHECK AND SAVE YOUR SETTINGS "));
    ShowMessageln(F("  FACTORY SETTING ARE USED INSTEAD  "));
    ShowMessageln(F("************************************"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }

  eereadwrite(readflag, addr, developerActive);
  eereadwrite(readflag, addr, motorAccel);
  eereadwrite(readflag, addr, motorSpeedMaxRpm);
  eereadwrite(readflag, addr, motorSpeedMaxPwm);
  eereadwrite(readflag, addr, motorPowerMax);
  eereadwrite(readflag, addr, motorSenseRightScale);
  eereadwrite(readflag, addr, motorSenseLeftScale);
  eereadwrite(readflag, addr, motorRollDegMax);
  eereadwrite(readflag, addr, motorRollDegMin);
  eereadwrite(readflag, addr, DistPeriOutRev);
  eereadwrite(readflag, addr, motorPowerIgnoreTime);
  eereadwrite(readflag, addr, motorForwTimeMax);
  eereadwrite(readflag, addr, motorMowSpeedMaxPwm);
  eereadwrite(readflag, addr, motorMowPowerMax);
  eereadwrite(readflag, addr, motorMowSpeedMinPwm);
  eereadwrite(readflag, addr, highGrassSpeedCoeff);
  eereadwrite(readflag, addr, motorLeftPID.Kp);
  eereadwrite(readflag, addr, motorLeftPID.Ki);
  eereadwrite(readflag, addr, motorLeftPID.Kd);
  eereadwrite(readflag, addr, motorMowPID.Kp);
  eereadwrite(readflag, addr, motorMowPID.Ki);
  eereadwrite(readflag, addr, motorMowPID.Kd);
  eereadwrite(readflag, addr, motorBiDirSpeedRatio1);
  eereadwrite(readflag, addr, motorBiDirSpeedRatio2);
  eereadwrite(readflag, addr, bumperUse);
  eereadwrite(readflag, addr, sonarUse);
  eereadwrite(readflag, addr, sonarCenterUse);
  eereadwrite(readflag, addr, sonarLeftUse);
  eereadwrite(readflag, addr, sonarRightUse);
  eereadwrite(readflag, addr, sonarTriggerBelow);
  eereadwrite(readflag, addr, perimeterUse);
  eereadwrite(readflag, addr, perimeterTriggerMinSmag);
  eereadwrite(readflag, addr, trackingErrorTimeOut);
  eereadwrite(readflag, addr, motorTickPerSecond);
  eereadwrite(readflag, addr, perimeterOutRevTime);
  eereadwrite(readflag, addr, perimeterTrackRollTime );
  eereadwrite(readflag, addr, perimeterTrackRevTime);
  eereadwrite(readflag, addr, perimeterPID.Kp);
  eereadwrite(readflag, addr, perimeterPID.Ki);
  eereadwrite(readflag, addr, perimeterPID.Kd);
  eereadwrite(readflag, addr, trakBlockInnerWheel);
  eereadwrite(readflag, addr, imuUse);
  eereadwrite(readflag, addr, stopMotorDuringCalib);
  eereadwrite(readflag, addr, imuDirPID.Kp);
  eereadwrite(readflag, addr, imuDirPID.Ki);
  eereadwrite(readflag, addr, imuDirPID.Kd);
  eereadwrite(readflag, addr, imuRollPID.Kp);
  eereadwrite(readflag, addr, imuRollPID.Ki);
  eereadwrite(readflag, addr, imuRollPID.Kd);
  eereadwrite(readflag, addr, remoteUse);
  eereadwrite(readflag, addr, batMonitor);
  eereadwrite(readflag, addr, batGoHomeIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfIdle);
  eereadwrite(readflag, addr, batFactor);  //float not use with ina226
  eereadwrite(readflag, addr, batChgFactor);  //float not use with ina226
  eereadwrite(readflag, addr, batSenseFactor); //float not use with ina226
  eereadwrite(readflag, addr, batFullCurrent);
  eereadwrite(readflag, addr, startChargingIfBelow);
  eereadwrite(readflag, addr, stationRevDist);
  eereadwrite(readflag, addr, stationRollAngle);
  eereadwrite(readflag, addr, stationForwDist);
  eereadwrite(readflag, addr, stationCheckDist);
  eereadwrite(readflag, addr, UseBumperDock);
  eereadwrite(readflag, addr, odometryTicksPerRevolution);
  eereadwrite(readflag, addr, odometryTicksPerCm);
  eereadwrite(readflag, addr, odometryWheelBaseCm);
  eereadwrite(readflag, addr, autoResetActive);
  eereadwrite(readflag, addr, CompassUse);
  eereadwrite(readflag, addr, twoWayOdometrySensorUse);   // char YES NO adress free for something else
  eereadwrite(readflag, addr, buttonUse);
  eereadwrite(readflag, addr, userSwitch1);
  eereadwrite(readflag, addr, userSwitch2);
  eereadwrite(readflag, addr, userSwitch3);
  eereadwrite(readflag, addr, timerUse);
  eereadwrite(readflag, addr, timer);
  eereadwrite(readflag, addr, rainUse);
  eereadwrite(readflag, addr, statsOverride);
  eereadwrite(readflag, addr, reduceSpeedNearPerimeter);
  eereadwrite(readflag, addr, autoAdjustSlopeSpeed);
  //eereadwriteString(readflag, addr, esp8266ConfigString);//string not used
  eereadwrite(readflag, addr, tiltUse);
  eereadwrite(readflag, addr, trackingPerimeterTransitionTimeOut);
  eereadwrite(readflag, addr, motorMowForceOff);
  eereadwrite(readflag, addr, MaxSpeedperiPwm);
  ActualSpeedPeriPWM = MaxSpeedperiPwm; //initialise Actual tracking speed
  eereadwrite(readflag, addr, RollTimeFor45Deg);  //unsigned long adress free for something else
  eereadwrite(readflag, addr, DistPeriObstacleAvoid);
  eereadwrite(readflag, addr, circleTimeForObstacle);
  eereadwrite(readflag, addr, DistPeriOutRev);
  eereadwrite(readflag, addr, motorRightOffsetFwd);
  eereadwrite(readflag, addr, motorRightOffsetRev);
  eereadwrite(readflag, addr, perimeterMagMaxValue);
  eereadwrite(readflag, addr, SpeedOdoMin);
  eereadwrite(readflag, addr, SpeedOdoMax);
  eereadwrite(readflag, addr, yawSet1);
  eereadwrite(readflag, addr, yawSet2);
  eereadwrite(readflag, addr, yawSet3);
  eereadwrite(readflag, addr, yawOppositeLane1RollRight);
  eereadwrite(readflag, addr, yawOppositeLane2RollRight);
  eereadwrite(readflag, addr, yawOppositeLane3RollRight);
  eereadwrite(readflag, addr, yawOppositeLane1RollLeft);
  eereadwrite(readflag, addr, yawOppositeLane2RollLeft);
  eereadwrite(readflag, addr, yawOppositeLane3RollLeft);
  eereadwrite(readflag, addr, DistBetweenLane);
  eereadwrite(readflag, addr, maxLenghtByLane);
  actualLenghtByLane = maxLenghtByLane; //initialise lenght lane
  eereadwrite(readflag, addr, swapCoilPolarityRight);
  eereadwrite(readflag, addr, read2Coil);
  eereadwrite(readflag, addr, maxDriftPerSecond);
  eereadwrite(readflag, addr, delayBetweenTwoDmpAutocalib);
  eereadwrite(readflag, addr, maxDurationDmpAutocalib);
  eereadwrite(readflag, addr, mowPatternDurationMax);
  eereadwrite(readflag, addr, DistPeriOutStop);
  eereadwrite(readflag, addr, Enable_Screen);
  eereadwrite(readflag, addr, RaspberryPIUse);
  //RaspberryPIUse=false;
  eereadwrite(readflag, addr, sonarToFrontDist);
  eereadwrite(readflag, addr, maxTemperature);
  eereadwrite(readflag, addr, dockingSpeed);
  eereadwrite(readflag, addr, rfidUse);
  eereadwrite(readflag, addr, compassRollSpeedCoeff);
  eereadwrite(readflag, addr, gpsUse);
  eereadwrite(readflag, addr, stuckIfGpsSpeedBelow);
  eereadwrite(readflag, addr, gpsSpeedIgnoreTime);
  eereadwrite(readflag, addr, useMqtt);
  eereadwrite(readflag, addr, stationHeading);
  eereadwrite(readflag, addr, checkDockingSpeed);
  eereadwrite(readflag, addr, batVoltageToStationStart);
  eereadwrite(readflag, addr, bumper_rev_distance);
  eereadwrite(readflag, addr, swapCoilPolarityLeft);
  eereadwrite(readflag, addr, useMotorDriveBrake);
  eereadwrite(readflag, addr, chargingMaxDuration);
  chargingTimeout = chargingMaxDuration * 3600000;// from hour to millis
  
  if (readflag)
  {
    ShowMessage(F("UserSettings OK from Address : "));
    ShowMessage(ADDR_USER_SETTINGS);
    ShowMessage(F(" To "));
    ShowMessageln(addr);
    motorInitialSpeedMaxPwm = motorSpeedMaxPwm; //the Pi can change the speed so store the initial value to restore after PFND for example
  } else {
    ShowMessage(F("UserSettings are saved from Address : "));
    ShowMessage(ADDR_USER_SETTINGS);
    ShowMessage(F(" To "));
    ShowMessageln(addr);
  }
}



void Robot::loadUserSettings() {
  //return; // use in one shot to reset all the usersetting if acces on console is not possible
  loadSaveUserSettings(true);
}



void Robot::printSettingSerial() {
  // ------- wheel motors ---------------------------------------------------------
  ShowMessageln("---------- wheel motors -----------");
  ShowMessage  ("motorAccel                 : ");
  ShowMessageln(motorAccel);
  ShowMessage  ("motorSpeedMaxRpm           : ");
  ShowMessageln(motorSpeedMaxRpm);
  ShowMessage  ("motorSpeedMaxPwm           : ");
  ShowMessageln(motorSpeedMaxPwm);
  ShowMessage  ("motorPowerMax              : ");
  ShowMessageln(motorPowerMax);
  ShowMessage  ("motorSenseRightScale       : ");
  ShowMessageln(motorSenseRightScale);
  ShowMessage  ("motorSenseLeftScale        : ");
  ShowMessageln(motorSenseLeftScale);
  //watchdogReset();
  ShowMessage  ("motorPowerIgnoreTime       : ");
  ShowMessageln(motorPowerIgnoreTime);
  ShowMessage  ("motorZeroSettleTime        : ");
  ShowMessageln(motorZeroSettleTime);
  ShowMessage  ("motorRollDegMax            : ");
  ShowMessageln(motorRollDegMax);
  ShowMessage  ("motorRollDegMin            : ");
  ShowMessageln(motorRollDegMin);
  ShowMessage  ("DistPeriOutRev             : ");
  ShowMessageln(DistPeriOutRev);
  //watchdogReset();
  ShowMessage  ("DistPeriOutStop            : ");
  ShowMessageln(DistPeriOutStop);
  ShowMessage  ("motorForwTimeMax           : ");
  ShowMessageln(motorForwTimeMax);
  ShowMessage  ("DistPeriObstacleAvoid      : ");
  ShowMessageln(DistPeriObstacleAvoid);
  ShowMessage  ("circleTimeForObstacle      : ");
  ShowMessageln(circleTimeForObstacle);
  ShowMessage  ("motorRightOffsetFwd        : ");
  ShowMessageln(motorRightOffsetFwd);
  //watchdogReset();
  ShowMessage  ("motorRightOffsetRev        : ");
  ShowMessageln(motorRightOffsetRev);
  ShowMessage  ("SpeedOdoMin                : ");
  ShowMessageln(SpeedOdoMin);
  ShowMessage  ("SpeedOdoMax                : ");
  ShowMessageln(SpeedOdoMax);
  ShowMessage  ("motorTickPerSecond         : ");
  ShowMessageln(motorTickPerSecond);

  ShowMessage  ("motorLeftPID.Kp            : ");
  ShowMessageln(motorLeftPID.Kp);
  ShowMessage  ("motorLeftPID.Ki            : ");
  ShowMessageln(motorLeftPID.Ki);
  ShowMessage  ("motorLeftPID.Kd            : ");
  ShowMessageln(motorLeftPID.Kd);

  ShowMessage  ("motorRightSwapDir          : ");
  ShowMessageln(motorRightSwapDir);
  ShowMessage  ("motorLeftSwapDir           : ");
  ShowMessageln(motorLeftSwapDir);
  ShowMessage  ("motorRightOffsetFwd        : ");
  ShowMessageln(motorRightOffsetFwd);
  ShowMessage  ("motorRightOffsetRev        : ");
  ShowMessageln(motorRightOffsetRev);
  ShowMessage  ("autoAdjustSlopeSpeed       : ");
  ShowMessageln(autoAdjustSlopeSpeed);
  //watchdogReset();
  delayWithWatchdog (500);
  // ------ mower motor -----------------------------------
  ShowMessageln("---------- mower motor -----------------");
  ShowMessage  ("motorMowForceOff         : ");
  ShowMessageln(motorMowForceOff);
  ShowMessage  ("motorMowAccel            : ");
  ShowMessageln(motorMowAccel);
  ShowMessage  ("motorMowSpeedMaxPwm      : ");
  ShowMessageln(motorMowSpeedMaxPwm);
  ShowMessage  ("(motorMowSpeedMinPwm     : ");
  ShowMessageln(motorMowSpeedMinPwm);
  ShowMessage  ("motorMowPowerMax         : ");
  ShowMessageln(motorMowPowerMax);
  ShowMessage  ("highGrassSpeedCoeff      : ");
  ShowMessageln(highGrassSpeedCoeff);
  //watchdogReset();
  
  // ------ bumper ------------------------------------
  ShowMessageln("---------- bumper -----------------");
  ShowMessage  ("bumperUse           : ");
  ShowMessageln(bumperUse);
  ShowMessage  ("bumper_rev_distance : ");
  ShowMessageln(bumper_rev_distance);
  
  // ------ rain -------------------------------------
  ShowMessageln("---------- rain ----------------");
  ShowMessage  ("rainUse             : ");
  ShowMessageln(rainUse);

  // ------  Temperature -----------------------
  ShowMessageln("----------  Temperature ---");
  ShowMessage  ("MaxTemperature     : ");
  ShowMessageln(maxTemperature);

  // ------ Screen -----------------------
  ShowMessage  ("Enable_Screen        : ");
  ShowMessageln(Enable_Screen);

  // ------ sonar -----------------------------------
  ShowMessageln(F("---------- sonar ---------------"));
  ShowMessage  ("sonarUse              : ");
  ShowMessageln(sonarUse);
  ShowMessage  ("sonarLikeBumper       : ");
  ShowMessageln(sonarLikeBumper);
  ShowMessage  ("sonarLeftUse        : ");
  ShowMessageln(sonarLeftUse);
  ShowMessage  ("sonarRightUse       : ");
  ShowMessageln(sonarRightUse);
  ShowMessage  ("sonarCenterUse      : ");
  ShowMessageln(sonarCenterUse);
  ShowMessage  ("sonarTriggerBelow   : ");
  ShowMessageln(sonarTriggerBelow);
  ShowMessage  ("sonarToFrontDist    : ");
  ShowMessageln(sonarToFrontDist);
  //watchdogReset();
  delayWithWatchdog (500);
  
  // ------ perimeter --------------------------
  ShowMessageln("---------- perimeter ------");
  ShowMessage  ("perimeterUse             : ");
  ShowMessageln(perimeterUse);
  ShowMessage  ("perimeterTriggerMinSmag  : ");
  ShowMessageln(perimeterTriggerMinSmag);
  ShowMessage  ("MaxSpeedperiPwm          : ");
  ShowMessageln(MaxSpeedperiPwm);
  ShowMessage  ("perimeterTrackRollTime   : ");
  ShowMessageln(perimeterTrackRollTime);
  ShowMessage  ("perimeterTrackRevTime    : ");
  ShowMessageln(perimeterTrackRevTime);
  ShowMessage  ("perimeterPID.Kp          : ");
  ShowMessageln(perimeterPID.Kp);
  ShowMessage  ("perimeterPID.Ki          : ");
  ShowMessageln( perimeterPID.Ki);
  //watchdogReset();
  ShowMessage  ("perimeterPID.Kd          : ");
  ShowMessageln(perimeterPID.Kd);
  ShowMessage  ("trackingPerimeterTransitionTimeOut: ");
  ShowMessageln(trackingPerimeterTransitionTimeOut);
  ShowMessage  ("trackingErrorTimeOut     : ");
  ShowMessageln(trackingErrorTimeOut);
  ShowMessage  ("perimeterMagMaxValue     : ");
  ShowMessageln(perimeterMagMaxValue);
  ShowMessage  ("swapCoilPolarityRight    : ");
  //watchdogReset();
  ShowMessageln(swapCoilPolarityRight);
  ShowMessage  ("swapCoilPolarityLeft     : ");
  ShowMessageln(swapCoilPolarityLeft);
  ShowMessage  ("read2Coil                : ");
  ShowMessageln(read2Coil);
  ShowMessage  ("trackingBlockInnerWheelWhilePerimeterStrug : ");
  ShowMessageln(trakBlockInnerWheel);
  ShowMessage  ("DistPeriOutRev           : ");
  ShowMessageln(DistPeriOutRev);
  ShowMessage  ("DistPeriObstacleRev      : ");
  ShowMessageln(DistPeriObstacleRev);
  ShowMessage  ("DistPeriOutForw          : ");
  ShowMessageln(DistPeriOutForw);
  ShowMessage  ("DistPeriObstacleForw     : ");
  ShowMessageln(DistPeriObstacleForw);
  //watchdogReset();
  delayWithWatchdog (500);
  
  // ------ By Lanes mowing ---------------------
  ShowMessageln(F("---------- By Lanes mowing ----------"));
  ShowMessage  (F("yawSet1                   : "));
  ShowMessageln(yawSet1);
  ShowMessage  (F("yawSet2                   : "));
  ShowMessageln(yawSet2);
  ShowMessage  (F("yawSet3                   : "));
  ShowMessageln(yawSet3);
  ShowMessage  (F("yawOppositeLane1RollRight : "));
  ShowMessageln(yawOppositeLane1RollRight);
  ShowMessage  (F("yawOppositeLane2RollRight : "));
  ShowMessageln(yawOppositeLane2RollRight);
  ShowMessage  (F("yawOppositeLane3RollRight : "));
  ShowMessageln(yawOppositeLane3RollRight);
  ShowMessage  (F("yawOppositeLane1RollLeft  : "));
  ShowMessageln(yawOppositeLane1RollLeft);
  //watchdogReset();
  ShowMessage  (F("yawOppositeLane2RollLeft  : "));
  ShowMessageln(yawOppositeLane2RollLeft);
  ShowMessage  (F("yawOppositeLane3RollLeft  : "));
  ShowMessageln(yawOppositeLane3RollLeft);
  ShowMessage  (F("DistBetweenLane           : "));
  ShowMessageln(DistBetweenLane);
  ShowMessage  (F("maxLenghtByLane           : "));
  ShowMessageln(maxLenghtByLane);
  //watchdogReset();
  
  // ------  IMU (compass/accel/gyro) ------
  ShowMessageln(F("---------- IMU (compass/accel/gyro) ---- "));
  ShowMessage  (F("imuUse                : "));
  ShowMessageln( imuUse);
  ShowMessage  (F("CompassUse            : "));
  ShowMessageln(CompassUse);
  ShowMessage  (F("stopMotorDuringCalib  : "));
  ShowMessageln(stopMotorDuringCalib);
  ShowMessage  (F("imuDirPID.Kp          : "));
  ShowMessageln(imuDirPID.Kp);
  ShowMessage  (F("imuDirPID.Ki          : "));
  ShowMessageln(imuDirPID.Ki);
  ShowMessage  (F("imuDirPID.Kd          : "));
  ShowMessageln( imuDirPID.Kd);
  //watchdogReset();
  ShowMessage  (F("maxDriftPerSecond     : "));
  ShowMessageln(maxDriftPerSecond);
  ShowMessage  (F("delayBetweenTwoDmpAutocalib : "));
  ShowMessageln(delayBetweenTwoDmpAutocalib);
  ShowMessage  (F("maxDurationDmpAutocalib     : "));
  ShowMessageln(maxDurationDmpAutocalib);
  ShowMessage  (F("compassRollSpeedCoeff       : "));
  ShowMessageln(compassRollSpeedCoeff);
  delayWithWatchdog (500);
  //watchdogReset();
  
  // ------ model R/C ------------------------------
  ShowMessageln(F("---------- model R/C ---------"));
  ShowMessage  (F("remoteUse                   : "));
  ShowMessageln(remoteUse);

  // ------ battery ----------------------------
  ShowMessageln(F("---------- battery --------  "));
  ShowMessage  (F("batMonitor           : "));
  ShowMessageln( batMonitor);
  ShowMessage  (F("batGoHomeIfBelow     : "));
  ShowMessageln(batGoHomeIfBelow);
  ShowMessage  (F("batSwitchOffIfBelow  : "));
  ShowMessageln(batSwitchOffIfBelow);
  ShowMessage  (F("batSwitchOffIfIdle   : "));
  ShowMessageln(batSwitchOffIfIdle);
  ShowMessage  (F("batFactor            : "));
  ShowMessageln( batFactor);
  ShowMessage  (F("batChgFactor         : "));
  ShowMessageln( batChgFactor);
  ShowMessage  (F("batFull              : "));
  ShowMessageln( batFull);
  ShowMessage  (F("batVoltageToStationStart: "));
  ShowMessageln(batVoltageToStationStart);
  ShowMessage  (F("batChargingCurrentMax: "));
  ShowMessageln(batChargingCurrentMax);
  ShowMessage  (F("batFullCurrent       : "));
  ShowMessageln(batFullCurrent);
  ShowMessage  (F("startChargingIfBelow : "));
  ShowMessageln(startChargingIfBelow);
  ShowMessage  (F("chargingTimeout      : "));
  ShowMessageln(chargingTimeout);
   ShowMessage  (F("chargingMaxDuration  : "));
  ShowMessageln(chargingMaxDuration);
  ShowMessage  (F("stationHeading       : "));
  ShowMessageln(stationHeading);
  ShowMessage  (F("batSenseFactor       : "));
  ShowMessageln( batSenseFactor);
  //ShowMessage  (F("chgSense             : "));
  //ShowMessageln(chgSense);
  //ShowMessage  (F("chgChange            : "));
  //ShowMessageln(chgChange);
  //ShowMessage  (F("chgNull              : "));
  //ShowMessageln(chgNull);
  //watchdogReset();
  // ------  charging station -----------------------------------------------------
  ShowMessageln(F("---------- charging station ----------------------------------"));
  ShowMessage  (F("stationRevDist     : "));
  ShowMessageln(stationRevDist);
  ShowMessage  (F("stationRollAngle   : "));
  ShowMessageln(stationRollAngle);
  ShowMessage  (F("stationForwDist    : "));
  ShowMessageln(stationForwDist);
  ShowMessage  (F("stationCheckDist   : "));
  ShowMessageln(stationCheckDist);
  ShowMessage  (F("UseBumperDock      : "));
  ShowMessageln(UseBumperDock);
  ShowMessage  (F("dockingSpeed       : "));
  ShowMessageln(dockingSpeed);
  ShowMessage (F("checkDockingSpeed  : "));
  ShowMessageln(checkDockingSpeed);
  ShowMessage  (F("autoResetActive    : "));
  ShowMessageln(autoResetActive);
  //watchdogReset();

  // ------ odometry --------------------------------------------------------------
  ShowMessageln(F("---------- odometry ------------------------------------------"));
  ShowMessage  (F("odometryTicksPerRevolution : "));
  ShowMessageln( odometryTicksPerRevolution);
  ShowMessage  (F("odometryTicksPerCm         : "));
  ShowMessageln( odometryTicksPerCm);
  ShowMessage  (F("odometryWheelBaseCm        : "));
  ShowMessageln( odometryWheelBaseCm);

  // ----- RFID ----------------------------------------------------------------------
  ShowMessageln(F("---------- RFID ----------- "));
  ShowMessage  (F("rfidUse         : "));
  ShowMessageln(rfidUse);
  //watchdogReset();
  
  // ----- RASPBERRY PI --------------
  ShowMessageln(F("---------- RASPBERRY PI------ "));
  ShowMessage  (F("RaspberryPIUse  : "));
  ShowMessageln(RaspberryPIUse);
  
  // ----- MQTT --------------
  ShowMessageln(F("---------- MQTT        ------ "));
  ShowMessage  (F("useMqtt  : "));
  ShowMessageln(useMqtt);

  // ----- GPS ----------------------------------------------------------------------
  ShowMessageln(F("---------- GPS -----------------------------------------------"));
  ShowMessage  (F("gpsUse                                     : "));
  ShowMessageln(gpsUse);
  ShowMessage  (F("stuckIfGpsSpeedBelow                       : "));
  ShowMessageln(stuckIfGpsSpeedBelow);
  ShowMessage  (F("gpsSpeedIgnoreTime                         : "));
  ShowMessageln(gpsSpeedIgnoreTime);

  // ----- other ----------------------------------------------------
  ShowMessageln(F("---------- other ------------"));
  ShowMessage  (F("buttonUse              : "));
  ShowMessageln(buttonUse);
  ShowMessage  (F("mowPatternDurationMax  : "));
  ShowMessageln(mowPatternDurationMax);
  //watchdogReset();

  // ----- user-defined switch ----------------------------------------
  ShowMessageln(F("---------- user-defined switch -------"));
  ShowMessage  (F("userSwitch1       : "));
  ShowMessageln(userSwitch1);
  ShowMessage  (F("userSwitch2       : "));
  ShowMessageln(userSwitch2);
  ShowMessage  (F("userSwitch3       : "));
  ShowMessageln(userSwitch3);
  //watchdogReset();
  
  // ----- timer --------------------------------------------------------------------
  ShowMessageln(F("---------- timer ----------- "));
  ShowMessage  (F("timerUse       : "));
  ShowMessageln(timerUse);
  //watchdogReset();
 
  // -------robot stats--------------------------------------------------------------
  ShowMessageln(F("---------- robot stats ---------------------------------------"));
  ShowMessage  (F("statsMowTimeMinutesTrip                    : "));
  ShowMessageln(statsMowTimeMinutesTrip);
  ShowMessage  (F("statsMowTimeMinutesTotal                   : "));
  ShowMessageln(statsMowTimeMinutesTotal);
  ShowMessage  (F("statsBatteryChargingCounterTotal           : "));
  ShowMessageln(statsBatteryChargingCounterTotal);
  ShowMessage  (F("statsBatteryChargingCapacityTrip in mAh    : "));
  ShowMessageln(statsBatteryChargingCapacityTrip);
  ShowMessage  (F("statsBatteryChargingCapacityTotal in Ah    : "));
  ShowMessageln(statsBatteryChargingCapacityTotal / 1000);
  ShowMessage  (F("statsBatteryChargingCapacityAverage in mAh : "));
  ShowMessageln(statsBatteryChargingCapacityAverage);
  //watchdogReset();
  //return;
}



void Robot::saveUserSettings() {
  ShowMessageln(F("START TO SAVE USER SETTINGS PLEASE WAIT"));
  loadSaveUserSettings(false);
}



void Robot::deleteUserSettings() {
  int addr = ADDR_USER_SETTINGS;
  ShowMessageln(F("ALL USER SETTINGS ARE DELETED PLEASE RESTART THE DUE"));
  eewrite(addr, (short)0); // magic
}



void Robot::deleteRobotStats() {
  statsMowTimeMinutesTrip = statsMowTimeMinutesTotal = statsBatteryChargingCounterTotal =
                              statsBatteryChargingCapacityTotal = statsBatteryChargingCapacityTrip = 0;
  loadSaveRobotStats(false);
  ShowMessageln(F("ALL ROBOT STATS ARE DELETED"));
}


// check robot stats
void Robot::checkRobotStats() {
  if (millis() < nextTimeRobotStats) return;
  nextTimeRobotStats = millis() + 60000;
  //----------------stats mow time------------------------------------------------------
  statsMowTimeHoursTotal = double(statsMowTimeMinutesTotal) / 60;
  if (statsMowTimeTotalStart) {
    statsMowTimeMinutesTripCounter++;
    statsMowTimeMinutesTrip = statsMowTimeMinutesTripCounter;
    statsMowTimeMinutesTotal++;
  }
  else if (statsMowTimeMinutesTripCounter != 0) {
    statsMowTimeMinutesTripCounter = 0;
  }
  //---------------stats Battery---------------------------------------------------------
  if ((stateCurr == STATE_STATION_CHARGING) && (stateTime >= 60000)) { // count only if mower is charged longer then 60sec
    statsBatteryChargingCounter++; // temporary counter

    if (statsBatteryChargingCounter == 1) statsBatteryChargingCounterTotal += 1;
    statsBatteryChargingCapacityTrip = batCapacity;
    statsBatteryChargingCapacityTotal += (batCapacity - lastTimeBatCapacity); // summ up only the difference between actual batCapacity and last batCapacity
    lastTimeBatCapacity = batCapacity;
  } else {                        
    // resets values to 0 when mower is not charging
    statsBatteryChargingCounter = 0;
    batCapacity = 0;
  }
  /*
    if (isnan(statsBatteryChargingCapacityTrip)) statsBatteryChargingCapacityTrip = 0;
    if (isnan(statsBatteryChargingCounterTotal)) statsBatteryChargingCounterTotal = 0; // for first run ensures that the counter is 0
    if (isnan(statsBatteryChargingCapacityTotal)) statsBatteryChargingCapacityTotal = 0; // for first run ensures that the counter is 0
  */
  if (statsBatteryChargingCapacityTotal <= 0 || statsBatteryChargingCounterTotal == 0) statsBatteryChargingCapacityAverage = 0; // make sure that there is no dividing by zero
  else statsBatteryChargingCapacityAverage = statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;
  //----------------new stats goes here------------------------------------------------------
  //mowPatternJustChange;
  mowPatternDuration++;
}

