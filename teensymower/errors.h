void Robot::loadSaveErrorCounters(boolean readflag) {
  if (readflag) {
    ShowMessage(F("Load ErrorData "));
  } else {
    ShowMessage(F("Save ErrorData "));
  }
  int addr = ADDR_ERR_COUNTERS;
  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    ShowMessageln(F("*****************************************"));
    ShowMessageln(F("EEPROM ERR COUNTERS: NO EEPROM ERROR DATA"));
    ShowMessageln(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    ShowMessageln(F("*****************************************"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, errorCounterMax);
  ShowMessage(F("Address Start= "));
  ShowMessage(ADDR_ERR_COUNTERS);
  ShowMessage(F(" Stop= "));
  ShowMessageln(addr);
}


void Robot::addErrorCounter(byte errType) {
  // increase error counters (both temporary and maximum error counters)
  if (errorCounter[errType] < 255) errorCounter[errType]++;
  if (errorCounterMax[errType] < 255) errorCounterMax[errType]++;
}



void Robot::resetErrorCounters() {
  ShowMessageln(F("resetErrorCounters"));
  for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = errorCounterMax[i] = 0;
  loadSaveErrorCounters(false);
  resetMotorFault();
}



void Robot::checkErrorCounter() {
  if (millis() >= nextTimeErrorCounterReset) {
    // reset all temporary error counters after 30 seconds (maximum error counters still continue to count)
    for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = 0;
    nextTimeErrorCounterReset = millis() + 30000; // 30 sec
  }
  if (stateCurr != STATE_OFF) {
    for (int i = 0; i < ERR_ENUM_COUNT; i++) {
      // set to fatal error if any temporary error counter reaches 10
      if (errorCounter[i] > 10) {
        ShowMessage("Error Counter > 10 for counter num ");
        ShowMessageln(i);
        setNextState(STATE_ERROR, 0);
      }
    }
  }
}