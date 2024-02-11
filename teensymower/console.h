void Robot::receivePiPfodCommand (String RpiCmd, float v1, float v2, float v3) {
  rc.processPI(RpiCmd, v1, v2, v3);
}

// print helpers
//void StreamPrint_progmem(Print &out,PGM_P format,...);
//#define Serialprint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)
#define Streamprint(stream,format, ...) StreamPrint_progmem(stream,PSTR(format),##__VA_ARGS__)
//String verToString(int v);


void StreamPrint_progmem(Print &out, PGM_P format, ...)
{
  // program memory version of printf - copy of format string and result share a buffer
  // so as to avoid too much memory use
  char formatString[128], *ptr;

  strncpy( formatString, format, sizeof(formatString) ); // copy in from program mem

  // null terminate - leave char since we might need it in worst case for result's \0
  formatString[ sizeof(formatString) - 2 ] = '\0';
  ptr = &formatString[ strlen(formatString) + 1 ]; // our result buffer...
  va_list args;
  va_start (args, format);
  vsnprintf(ptr, sizeof(formatString) - 1 - strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString) - 1 ] = '\0';
  out.print(ptr);
}


void Robot::printInfo(Stream & s) {
  if ((consoleMode == CONSOLE_OFF) || (consoleMode == CONSOLE_TRACKING)) {
  } else {

    Streamprint(s, "t%6u ", (millis() - stateStartTime) / 1000);
    Streamprint(s, "Loops%7u ", loopsPerSec);

    Streamprint(s, "v%1d ", consoleMode);

    Streamprint(s, "%4s ", stateNames[stateCurr]);

    if (consoleMode == CONSOLE_PERIMETER) {
      Streamprint(s, "sig min %4d max %4d avg %4d mag %5d qty %3d",
                  (int)perimeter.getSignalMin(0), (int)perimeter.getSignalMax(0), (int)perimeter.getSignalAvg(0),
                  perimeterMagLeft, (int)(perimeter.getFilterQuality(0) * 100.0));
      Streamprint(s, "  in %2d  cnt %4d  on %1d\r\n",
                  (int)perimeterInsideLeft, perimeterCounter, (int)(!perimeter.signalTimedOut(0)) );
    } else {
      Streamprint(s, "odo %4d %4d ", (int)odometryLeft, (int)odometryRight);
      Streamprint(s, "spd %4d %4d %4d ", (int)motorLeftSpeedRpmSet, (int)motorRightSpeedRpmSet, (int)motorMowPwmCoeff);
      if (consoleMode == CONSOLE_SENSOR_VALUES) {
        // sensor values
        Streamprint(s, "sen %4d %4d %4d ", (int)motorLeftPower, (int)motorRightPower, (int)motorMowPower);
        Streamprint(s, "bum %4d %4d ", bumperLeft, bumperRight);

        Streamprint(s, "son %4d %4d %4d ", sonarDistLeft, sonarDistCenter, sonarDistRight);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        if (perimeterUse) Streamprint(s, "per %3d ", (int)perimeterInsideLeft);
      } else {
        // sensor counters
        Streamprint(s, "sen %4d %4d %4d ", motorLeftSenseCounter, motorRightSenseCounter, motorMowSenseCounter);
        Streamprint(s, "bum %4d %4d ", bumperLeftCounter, bumperRightCounter);

        //Streamprint(s, "son %3d ", sonarDistCounter);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        //Streamprint(s, "per %3d ", perimeterLeft);
        if (perimeterUse) Streamprint(s, "per %3d ", perimeterCounter);
        //if (gpsUse) Streamprint(s, "gps %2d ", (int)gps.satellites());
      }
      Streamprint(s, "bat %2d.%01d ", (int)batVoltage, (int)((batVoltage * 10) - ((int)batVoltage * 10)) );
      Streamprint(s, "chg %2d.%01d %2d.%01d ",
                  (int)chgVoltage, (int)((chgVoltage * 10) - ((int)chgVoltage * 10)),
                  (int)chgCurrent, (int)((abs(chgCurrent) * 10) - ((int)abs(chgCurrent) * 10))
                 );
      //Streamprint(s, "imu%3d ", imu.getCallCounter());
      //   Streamprint(s, "adc%3d ", ADCMan.getCapturedChannels());
      Streamprint(s, "%s\r\n", name.c_str());
    }
  }
}



void Robot::printMenu() {
  Serial.println(" ");
  Serial.println(F(" MAIN MENU:"));
  Serial.println(F("1=test motors"));
  Serial.println(F("To test odometry --> use Arduremote"));
  Serial.println(F("3=communications menu"));
  Serial.println(F("5=Deactivate and Delete GYRO calibration : To calibrate GYRO --> use Arduremote Do not move IMU during the Calib"));
  Serial.println(F("6=Deactivate and Delete Compass calibration : To calibrate Compass --> use Arduremote start/stop"));
  Serial.println(F("9=save user settings"));
  Serial.println(F("l=load factory settings: Do not save setting before restart the mower"));
  Serial.println(F("r=delete robot stats"));
  Serial.println(F("x=read settings"));
  Serial.println(F("e=delete all errors"));
  Serial.println(F("0=exit"));
  Serial.println(" ");
}



void Robot::menu() {
  char ch;
  printMenu();
  while (true) {
    //watchdogReset();
    resetIdleTime();
    // imu.update();
    if ((!RaspberryPIUse) && (Serial.available() > 0)) {  //do not read console is raspberry connected
      ch = (char)Serial.read();
      switch (ch) {
        case '0':
          nextTimeInfo = millis();
          return;

        case '1':
          testMotors();
          printMenu();
          break;

        case '5':
          //     imu.deleteAccelGyroCalib();
          imuUse = false;
          printMenu();
          break;

        case '6':
          //       imu.deleteCompassCalib();
          CompassUse = false;
          printMenu();
          break;

        case '9':
          saveUserSettings();
          printMenu();
          break;

        case 'l':
          printSettingSerial();
          deleteUserSettings();
          printMenu();
          break;

        case 'r':
          printSettingSerial();
          deleteRobotStats();
          printMenu();
          break;

        case 'x':
          printSettingSerial();
          ShowMessageln(F("DONE"));
          printMenu();
          break;

        case 'e':
          resetErrorCounters();
          setNextState(STATE_OFF, 0);
          ShowMessageln(F("ALL ERRORS ARE DELETED"));
          printMenu();
          break;
      }
    }
    delay(10);
  }
}


void Robot::printOdometry() {
  ShowMessage(F("ODO,"));
  ShowMessage(odometryX);
  ShowMessage(",");
  ShowMessageln(odometryY);
  ShowMessage(F("ODO,"));
  ShowMessage(odometryX);
  ShowMessage(",");
  ShowMessageln(odometryY);
}



void Robot::readSerial() {
  if ((!RaspberryPIUse) && (Serial.available() > 0)) {  //do not read console if raspberry connected
    char ch = (char)Serial.read();
    //resetIdleTime();
    switch (ch) {
      case '0':
        // press '0' for OFF
        setNextState(STATE_OFF, 0);
        break;

      case '1':
        // press '1' for Automode
        //motorMowEnable = true;
        setNextState(STATE_ACCEL_FRWRD, 0);
        break;

      case 'd':
        menu(); // menu
        break;

      case 'h':
        setNextState(STATE_PERI_FIND, 0); // press 'h' to drive home
        break;

      case 'l':
        bumperLeft = true; // press 'l' to simulate left bumper
        bumperLeftCounter++;
        break;

      case 'q':
        yawCiblePos = 90;
        setNextState(STATE_ROLL_TO_FIND_YAW, 0); // press 'h' to drive home
        break;

      case 'r':
        //setBeeper(400, 50, 50, 200, 0 );//error
        break;

      case 's':
        //imu.calibComStartStop();
        break;

      case 't':
        setNextState(STATE_PERI_TRACK, 0); // press 't' to track perimeter
        break;

      case 'u':
        setNextState(STATE_ACCEL_FRWRD, RIGHT);
        break;

      case 'v':
        //bb
        consoleMode = (consoleMode + 1) % 5;
        ShowMessageln(consoleModeNames[consoleMode]);
        break;
    }
  }
}