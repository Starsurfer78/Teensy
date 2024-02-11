// check battery voltage and decide what to do
void Robot::checkBattery() {
  if ((millis() < nextTimeCheckBattery) || (millis() < 30000)) return; //  wait 30 sec after the initial power on before first check to avoid read bad battery voltage
  nextTimeCheckBattery = millis() + 1000; //if change need to adjust the line idleTimeSec= idleTimeSec+1;

  if (batMonitor) {
    // if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_OFF) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING))  {
    if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_OFF))   {
      ShowMessage(F("Batterie Voltage : "));
      ShowMessage(batVoltage);
      ShowMessage(F(" -- > Switch OFF Voltage : "));
      ShowMessage(batSwitchOffIfBelow);
      ShowMessageln(F("  Bat Voltage is very low the state is changed to OFF, so the undervoltage timer start"));
      addErrorCounter(ERR_BATTERY);
      setBeeper(3000, 500, 0, 2000, 0);//beep for 3 sec
      setNextState(STATE_OFF, 0);
    }
    else if ((batVoltage < batGoHomeIfBelow) && (stateCurr == STATE_FORWARD_ODO) && (perimeterUse)) {    //actualy in mowing mode with station and perimeter
      ShowMessage(F("Batterie Voltage : "));
      ShowMessage(batVoltage);
      ShowMessage(F(" -- > Minimum Mowing Voltage : "));
      ShowMessageln(batGoHomeIfBelow);
      ShowMessageln(F(" Bat Voltage is low : The mower search the charging Station"));
      setBeeper(3000, 500, 100, 2000, 100);//beep for 3 sec
      statusCurr = BACK_TO_STATION;
      areaToGo = 1;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      periFindDriveHeading = imu.ypr.yaw;
      setNextState(STATE_PERI_FIND, 0);
    }
    // if robot is OFF or Error  we can start to count before shutdown
    if ( (stateCurr == STATE_OFF) || (stateCurr == STATE_ERROR)) {
      /*
            ShowMessage("Count before power OFF  ");
            ShowMessage(idleTimeSec);
            ShowMessage(" / ");
            ShowMessageln(batSwitchOffIfIdle * 60);
      */
      if (idleTimeSec != BATTERY_SW_OFF) { // battery already switched off?
        idleTimeSec = idleTimeSec + 1; // this loop is execute each 1 seconde so add 1 second to idle time
        if (idleTimeSec > batSwitchOffIfIdle * 60) {

          if (RaspberryPIUse) {
            ShowMessageln(F("Battery IDLE trigger "));
            ShowMessageln(F("PCB power OFF after 30 secondes Wait Until PI Stop "));
            MyRpi.sendCommandToPi("PowerOffPi");
            delayWithWatchdog(30000);//wait 30Sec  until pi is OFF or the USB native power again the due and the undervoltage never switch OFF
          } else {
            ShowMessageln(F("PCB power OFF immediatly"));
          }
          setBeeper(3000, 3000, 0, 2000, 0);
          loadSaveErrorCounters(false); // saves error counters
          loadSaveRobotStats(false);    // saves robot stats
          idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
          ShowMessageln(F("BATTERY switching OFF"));
          delayWithWatchdog(2000);
          Serial1.end();//disconnect BT
          delayWithWatchdog(3000);
          setActuator(ACT_BATTERY_SW, 0);  // switch off battery
        }
      }
    } else {
      resetIdleTime();
    }
  }
}