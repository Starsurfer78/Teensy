//char* Robot::rfidToDoName() {
//  return rfidToDoNames[rfidToDoCurr];
//}

const char* Robot::rfidToDoName() {
    // Kopie der Zeichenkette erstellen und zurückgeben
    static char buffer[32]; // Größe an deine Anforderungen anpassen
    strcpy(buffer, rfidToDoNames[rfidToDoCurr]);
    return buffer;
}


char* Robot::rfidToDoNameList(byte rfidToDoIndex) {
  return rfidToDoNames[rfidToDoIndex];
}



boolean Robot::search_rfid_list(unsigned long TagNr) {
  boolean tag_exist_in_list = false;
  ptr = head;
  if (ptr != NULL) {
    for (ptr = head; ptr->next != NULL; ptr = ptr->next) {
      if (ptr->TagNr == TagNr) tag_exist_in_list = true;
    }
  }
  return tag_exist_in_list;
}


void Robot::rfidTagTraitement(unsigned long TagNr, byte statusCurr) {
  boolean tagAndStatus_exist_in_list = false;
  String line01 = "";
  //struct rfid_list *temp = (struct rfid_list*) malloc(sizeof(rfid_list));
  ptr = head;
  if (ptr != NULL) {
    for (ptr = head; ptr->next != NULL; ptr = ptr->next) {
      if ((ptr->TagNr == TagNr) && (ptr->TagMowerStatus == statusCurr)) {
        tagAndStatus_exist_in_list = true;
        //ptr is locate on the correct record in the list --> exit from the for loop
        break;
      }
    }
  }
  if (tagAndStatus_exist_in_list) {
    //debut du traitement
    ShowMessage(F("Tag and Status find to do is "));
    ShowMessageln(F(rfidToDoNameList(ptr->TagToDo)));
    switch (ptr->TagToDo) {

      case NOTHING:
        ShowMessageln(F("nothing to do ???"));
        break;

      case RTS:
        ShowMessage("Fast return tag : Turning ");
        ShowMessage(ptr->TagAngle1);
        ShowMessage(" degrees and new speed is ");
        ShowMessageln(ptr->TagSpeed);
        newtagRotAngle1 = ptr->TagAngle1;
        motorSpeedMaxPwm = ptr->TagSpeed;
        setNextState(STATE_PERI_STOP_TOROLL, 0);
        break;

      case FAST_START:
        ShowMessage("Faster start tag. Turning ");
        ShowMessage(ptr->TagAngle1);
        ShowMessage("° and new speed is ");
        ShowMessageln(ptr->TagSpeed);
        if (areaToGo != 1) { // if a distance is set for start point we can't use the fast start
          newtagRotAngle1 = ptr->TagAngle1;
          motorSpeedMaxPwm = ptr->TagSpeed;
          setNextState(STATE_PERI_STOP_TO_FAST_START, 0);
        } else {
          ShowMessageln("Fast start is only valid to change mowing area");
        }
        break;

      case NEW_AREA:
        ShowMessageln("Not use better to use AREA1,2 or 3");
        //not use
        break;

      case AREA1:
        startStopSender(1, 1);
        startStopSender(2, 0);
        startStopSender(3, 0);
        areaToGo = 1;
        ShowMessageln("Return to Station area ");
        motorSpeedMaxPwm = ptr->TagSpeed;
        newtagRotAngle1 = ptr->TagAngle1;
        newtagDistance1 = ptr->TagDist1;
        newtagRotAngle2 = ptr->TagAngle2;
        newtagDistance2 = ptr->TagDist2;
        setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
        //#stopsender
        if (areaInMowing == 2) {
          //ButtonStopArea2_click()
        }
        if (areaInMowing == 3) {
          //ButtonStopArea3_click()
        }
        break;

      case AREA2:
        //send data to ESP32 to start AREA2 sender and stop AREA1 one
        line01 = "#SENDER," + area1_ip + ",A0";
        Bluetooth.println(line01);
        line01 = "#SENDER," + area2_ip + ",B1";
        Bluetooth.println(line01);
        if (areaToGo == 2) {
          ShowMessageln("Go to AREA2");
          motorSpeedMaxPwm = ptr->TagSpeed;
          newtagRotAngle1 = ptr->TagAngle1;
          newtagDistance1 = ptr->TagDist1;
          newtagRotAngle2 = ptr->TagAngle2;
          newtagDistance2 = ptr->TagDist2;
          setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
        }
        break;

      case AREA3:
        line01 = "#SENDER," + area1_ip + ",A0";
        Bluetooth.println(line01);
        line01 = "#SENDER," + area3_ip + ",B1";
        Bluetooth.println(line01);
        if (areaToGo == 3) {
          ShowMessageln("Go to AREA3");
          motorSpeedMaxPwm = ptr->TagSpeed;
          newtagRotAngle1 = ptr->TagAngle1;
          newtagDistance1 = ptr->TagDist1;
          newtagRotAngle2 = ptr->TagAngle2;
          newtagDistance2 = ptr->TagDist2;
          setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
        }
        break;

      case SPEED:
        ActualSpeedPeriPWM = ptr->TagSpeed;
        newtagDistance1 = ptr->TagDist1;
        whereToResetSpeed =  totalDistDrive + newtagDistance1; // when a speed tag is read it's where the speed is back to maxpwm value
        ShowMessage("Change to speed  : ");
        ShowMessage(ActualSpeedPeriPWM);
        ShowMessage(" for next ");
        ShowMessage(newtagDistance1);
        ShowMessageln(" centimeters");
        break;
    }
  } else {
    ShowMessageln(F("Tag and Status not match"));
  }
}


void Robot::insert_rfid_list(unsigned long TagNr, byte TagMowerStatus, byte TagToDo, int TagSpeed, float TagAngle1, int TagDist1, float TagAngle2, int TagDist2) {
  struct rfid_list *node = (struct rfid_list*) malloc(sizeof(*node));//allocation dynamique de la memoire
  if (node == NULL) {
    ShowMessageln(F("New Rfid tag list insert error "));
    return;
  }

  node->TagNr = TagNr;
  node->TagMowerStatus = TagMowerStatus;
  node->TagToDo = TagToDo;
  node->TagSpeed = TagSpeed;
  node->TagAngle1 = TagAngle1;
  node->TagDist1 = TagDist1;
  node->TagAngle2 = TagAngle2;
  node->TagDist2 = TagDist2;
  node->next = head;  // def du nouveau noeud au premier
  head = node; // tete de la liste devient celui que l on a ajouté.
  rfidListElementCount = rfidListElementCount + 1;
  ShowMessageln(F("1 RFID TAG insertion OK"));
  ShowMessage(F("NEW RFID LIST COUNT = "));
  ShowMessageln(rfidListElementCount);
}



void Robot::delete_rfid_list(unsigned long TagNr, byte TagMowerStatus, int pos_into_list) {
  struct rfid_list *supp_element = NULL;
  ShowMessage(F("Delete element Nr: "));
  ShowMessageln(int(pos_into_list));
  ptr = head;  // move at the beginning of the list
  for (int i = 1; i < pos_into_list; ++i) {
    ptr = ptr->next; // move just before the one to delete
  }
  supp_element = ptr->next; //  the one to delete
  ptr->next = ptr->next->next; //  rewrite the pointer of element before the supress one to the next next one
  // if(ptr->next == NULL)
  //         liste->fin = courant;
  //free (supp_element->donnee);
  free (supp_element);  // free memory  need maybe more free for detail element
  rfidListElementCount = rfidListElementCount - 1;
  ShowMessageln(F("1 RFID TAG suppression OK"));
  ShowMessage(F("NEW RFID LIST COUNT = "));
  ShowMessageln(rfidListElementCount);
}



void Robot::sort_rfid_list() {
  struct rfid_list *p = NULL;
  struct rfid_list PR ;
  struct rfid_list *temp = (struct rfid_list*) malloc(sizeof(rfid_list));
  ptr = head;
  if (ptr != NULL) {
    for (temp = head; temp->next != NULL; temp = temp->next) {
      for (p = temp->next; p != NULL; p = p->next) {

        if (p->TagNr < temp->TagNr) {
          PR.TagNr = p->TagNr;
          PR.TagMowerStatus = p->TagMowerStatus;
          PR.TagToDo = p->TagToDo;
          PR.TagSpeed = p->TagSpeed;
          PR.TagAngle1 = p->TagAngle1;
          PR.TagDist1 = p->TagDist1;
          PR.TagAngle2 = p->TagAngle2;
          PR.TagDist2 = p->TagDist2;
          //p->TagNr = temp->TagNr;
          p->TagNr = temp->TagNr;
          p->TagMowerStatus = temp->TagMowerStatus;
          p->TagToDo = temp->TagToDo;
          p->TagSpeed = temp->TagSpeed;
          p->TagAngle1 = temp->TagAngle1;
          p->TagDist1 = temp->TagDist1;
          p->TagAngle2 = temp->TagAngle2;
          p->TagDist2 = temp->TagDist2;
          //temp->TagNr = PR.TagNr;
          temp->TagNr = PR.TagNr;
          temp->TagMowerStatus = PR.TagMowerStatus;
          temp->TagToDo = PR.TagToDo;
          temp->TagSpeed = PR.TagSpeed;
          temp->TagAngle1 = PR.TagAngle1;
          temp->TagDist1 = PR.TagDist1;
          temp->TagAngle2 = PR.TagAngle2;
          temp->TagDist2 = PR.TagDist2;
        }
      }
    }
  }
}




void Robot::print_rfid_list() {
  ShowMessageln("RFID LIST :");
  ptr = head;
  //struct rfid_list *ptr = head;
  // rfid todo list
  //enum { NOTHING, RTS, FAST_START, NEW_AREA, SPEED, AREA1, AREA2, AREA3 };
  while (ptr != NULL) {  //parcours jusqu au dernier
    ShowMessage(String(ptr->TagNr, HEX));
    ShowMessage(",");
    ShowMessage(statusNames[ptr->TagMowerStatus]);
    ShowMessage(",");
    ShowMessage(rfidToDoNames[ptr->TagToDo]);
    ShowMessage(",");
    ShowMessage(ptr->TagSpeed);
    ShowMessage(",");
    ShowMessage(ptr->TagAngle1);
    ShowMessage(",");
    ShowMessage(ptr->TagDist1);
    ShowMessage(",");
    ShowMessage(ptr->TagAngle2);
    ShowMessage(",");
    ShowMessageln(ptr->TagDist2);
    ptr = ptr->next;
  }
}



void Robot::saveRfidList() {
  boolean readflag = false;
  int addr = ADDR_RFID_LIST;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  ShowMessage("RFID LIST COUNT = ");
  ShowMessageln(rfidListElementCount);
  eereadwrite(readflag, addr, rfidListElementCount); // magic
  ptr = head;
  while (ptr != NULL) {  //parcours jusqu au dernier
    eereadwrite(readflag, addr, ptr->TagNr);
    eereadwrite(readflag, addr, ptr->TagMowerStatus);
    eereadwrite(readflag, addr, ptr->TagToDo);
    eereadwrite(readflag, addr, ptr->TagSpeed);
    eereadwrite(readflag, addr, ptr->TagAngle1);
    eereadwrite(readflag, addr, ptr->TagDist1);
    eereadwrite(readflag, addr, ptr->TagAngle2);
    eereadwrite(readflag, addr, ptr->TagDist2);
    ptr = ptr->next;
  }
  ShowMessage(F("RFID LIST address Start="));
  ShowMessageln(ADDR_RFID_LIST);
  ShowMessage(F("RFID LIST address Stop="));
  ShowMessageln(addr);
}



void Robot::loadRfidList() {
  byte rfidListElementTotal = 0;
  boolean readflag = true;
  int addr = ADDR_RFID_LIST;
  struct rfid_list PR ;
  short magic = 0;
  eereadwrite(readflag, addr, magic); // magic
  if (magic != MAGIC) {
    ShowMessageln(F("RFID LIST USERDATA: NO EEPROM RFID LIST DATA"));
    ShowMessageln(F("PLEASE SAVE YOUR RFID LIST ONCE"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, rfidListElementTotal); // magic
  ShowMessage("rfidListElementTotal = ");
  ShowMessageln(rfidListElementTotal);
  for (int i = 0; i < rfidListElementTotal; i++) {
    ShowMessage(i);
    ShowMessageln ("TAG READ");
    eereadwrite(readflag, addr, PR.TagNr);
    eereadwrite(readflag, addr, PR.TagMowerStatus);
    eereadwrite(readflag, addr, PR.TagToDo);
    eereadwrite(readflag, addr, PR.TagSpeed);
    eereadwrite(readflag, addr, PR.TagAngle1);
    eereadwrite(readflag, addr, PR.TagDist1);
    eereadwrite(readflag, addr, PR.TagAngle2);
    eereadwrite(readflag, addr, PR.TagDist2);
    insert_rfid_list(PR.TagNr, PR.TagMowerStatus, PR.TagToDo, PR.TagSpeed, PR.TagAngle1, PR.TagDist1, PR.TagAngle2, PR.TagDist2);
  }
  ShowMessage(F("RFID LIST address Start="));
  ShowMessageln(ADDR_RFID_LIST);
  ShowMessage(F("RFID LIST address Stop="));
  ShowMessageln(addr);
  sort_rfid_list();
  print_rfid_list();
}



void Robot::newTagFind() {
  if (millis() >= nextTimeSendTagToPi) {
    nextTimeSendTagToPi = millis() + 10000;
    ShowMessage("Find a tag : ");
    ShowMessageln(rfidTagFind);
    unsigned long rfidTagFind_long = hstol(rfidTagFind);

    if (rfidUse) {
      if (search_rfid_list(rfidTagFind_long)) {
        rfidTagTraitement(rfidTagFind_long, statusCurr);
      } else {
        ShowMessage("Auto insert Wait tag : ");
        ShowMessageln(rfidTagFind);
        insert_rfid_list(rfidTagFind_long , 0, 0, 100, 1, 1, 1, 1);
        sort_rfid_list();
      }
    }
  }
}