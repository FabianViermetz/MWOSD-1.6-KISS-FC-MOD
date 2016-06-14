
#if defined NAZA
  #define SERIALBUFFERSIZE 75
#elif defined GPSOSD
  #define SERIALBUFFERSIZE 100
#elif defined KISS_FC // TLM message is 154 bytes long
  #define SERIALBUFFERSIZE 154
#else
  #define SERIALBUFFERSIZE 150
#endif

static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint32_t KSP_rcvChecksum; // uint32_t needed for KISS
static uint8_t MSP_rcvChecksum;
static uint8_t readIndex;
uint8_t txChecksum;

//#ifdef KISS_FC
//int32_t KSP_read32() {
//  int32_t t = KSP_read16()<<16;
//  t |= (int32_t)KSP_read16();
//  return t;
//}
//
//int16_t KSP_read16() {
//  int16_t t = KSP_read8()<<8;
//  t |= (int16_t)KSP_read8();
//  return t;
//}
//
//int8_t KSP_read8()  {
//  return serialBuffer[readIndex++];
//}
//
//#endif // KISS_FC

uint32_t MSP_read32() {
  uint32_t t = MSP_read16();
  t |= (uint32_t)MSP_read16()<<16;
  return t;
}

uint16_t MSP_read16() {
  uint16_t t = read8();
  t |= (uint16_t)read8()<<8;
  return t;
}

uint8_t read8()  {
  return serialBuffer[readIndex++];
}

void WriteGerneralRequest(uint8_t mspCommand, uint8_t txDataSize){
  if (KISS_FC_connected) {
    if (mspCommand == MSP_ATTITUDE) {
      kspWriteRequest(KSP_TLM, txDataSize);
    }
  }
  else mspWriteRequest(mspCommand, txDataSize);
}

void mspWriteRequest(uint8_t mspCommand, uint8_t txDataSize){
  Serial.write('$');
  Serial.write('M');
  Serial.write('<');
  txChecksum = 0;
  mspWrite8(txDataSize);
  mspWrite8(mspCommand);
  if(txDataSize == 0)
    mspWriteChecksum();
}

void kspWriteRequest(uint8_t mspCommand, uint8_t txDataSize) {
  Serial.write(mspCommand);
}

void mspWrite8(uint8_t t){
  Serial.write(t);
  txChecksum ^= t;
}

#ifdef KISS_FC
void kspWrite16(uint16_t t){
  mspWrite8(t>>8);
  mspWrite8(t & 0xFF);
}
#endif

void mspWrite16(uint16_t t){
  mspWrite8(t);
  mspWrite8(t>>8);
}

void mspWriteChecksum(){
  Serial.write(txChecksum);
}
// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;
  #ifdef MSPACTIVECHECK
    timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
  #endif
  if (cmdMSP == MSP_OSD) {
    uint8_t cmd = read8();

    if (cmd == OSD_READ_CMD_EE) {
      eeaddress = read8();
      eeaddress = eeaddress+read8();
      eedata = read8();
      settingsMode=1;
      MSP_OSD_timer=3000+millis();
      settingsSerialRequest();
    }

    if (cmd == OSD_WRITE_CMD_EE) {
      for(uint8_t i=0; i<10; i++) {
        eeaddress = read8();
        eeaddress = eeaddress+(read8()<<8);
        eedata = read8();
        settingsMode=1;
        MSP_OSD_timer=3000+millis();
        EEPROM.write(eeaddress,eedata);
//        if (eeaddress==0){
          EEPROM.write(0,MWOSDVER);
//        }
        if ((eeaddress==EEPROM_SETTINGS+(EEPROM16_SETTINGS*2))||(eeaddress==EEPROM_SETTINGS+(EEPROM16_SETTINGS*2)+(3*2*POSITIONS_SETTINGS))){
          readEEPROM();
        }
      }
      eeaddress++;
    settingswriteSerialRequest();
    }
#ifdef GUISENSORS
    if (cmd == OSD_SENSORS) {
      WriteGerneralRequest(MSP_OSD,1+10);
      mspWrite8(OSD_SENSORS);
      for (uint8_t sensor=0;sensor<SENSORTOTAL;sensor++) {
//        uint16_t sensortemp = analogRead(sensorpinarray[sensor]);
        uint16_t sensortemp = (uint16_t)sensorfilter[sensor][SENSORFILTERSIZE]/SENSORFILTERSIZE;
        mspWrite16(sensortemp);
      }
       mspWriteChecksum();
    }
#endif

    if(cmd == OSD_GET_FONT) {
      if(dataSize == 5) {
        if(MSP_read16() == 7456) {
          nextCharToRequest = read8();
          lastCharToRequest = read8();
          initFontMode();
        }
      }
      else if(dataSize == 56) {
        for(uint8_t i = 0; i < 54; i++)
          fontData[i] = read8();
      
	uint8_t c = read8();
        write_NVM(c);
	//fontCharacterReceived(c);
        if (c==255)
          MAX7456Setup();
      }
    }
    if(cmd == OSD_DEFAULT) {
      EEPROM_clear(); 
      checkEEPROM();
      flags.reset=1;
    }
    if(cmd == OSD_RESET) {
        flags.reset=1;
    }
                    
  }

#ifdef PROTOCOL_MSP
#ifndef GPSOSD
  if (cmdMSP==MSP_IDENT)
  {
    flags.ident=1;
    MwVersion= read8();                             // MultiWii Firmware version
  }

  if (cmdMSP==MSP_STATUS)
  {
    cycleTime=MSP_read16();
    I2CError=MSP_read16();
    MwSensorPresent = MSP_read16();
    MwSensorActive = MSP_read32();
    #if defined FORCESENSORS
      MwSensorPresent=GPSSENSOR|BAROMETER|MAGNETOMETER|ACCELEROMETER;
    #endif  
    armed = (MwSensorActive & mode.armed) != 0;
    FCProfile = read8();
    if (!configMode){
      CurrentFCProfile=FCProfile;
      PreviousFCProfile=FCProfile;
     }
  }

  if (cmdMSP==MSP_RC)
  {
    for(uint8_t i=0;i<8;i++)
      MwRcData[i] = MSP_read16();
    handleRawRC();
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    #ifdef GPSACTIVECHECK
     timer.GPS_active=GPSACTIVECHECK;
    #endif //GPSACTIVECHECK
    uint8_t GPS_fix_temp=read8();
    if (GPS_fix_temp){
      GPS_fix=1;
    }
    GPS_numSat=read8();
    GPS_latitude = MSP_read32();
    GPS_longitude = MSP_read32();
    GPS_altitude = MSP_read16();
    #if defined RESETGPSALTITUDEATARM
      if (!armed){
        GPS_home_altitude=GPS_altitude;
      } 
      GPS_altitude=GPS_altitude-GPS_home_altitude;
    #endif // RESETGPSALTITUDEATARM  
    #if defined I2CGPS_SPEED
      GPS_speed = MSP_read16()*10;
      //gpsfix(); untested
    #else
      GPS_speed = MSP_read16();
    #endif // I2CGPS_SPEED
    GPS_ground_course = MSP_read16();
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    GPS_distanceToHome=MSP_read16();
#ifdef I2CGPS_DISTANCE
    gpsdistancefix();
#endif
    
    GPS_directionToHome=MSP_read16();
#ifdef GPSTIME
    read8(); //missing
    GPS_time = MSP_read32();        //local time of coord calc - haydent
#endif
  }

#if defined MULTIWII_V24
  if (cmdMSP==MSP_NAV_STATUS)
  {
     read8();
     read8();
     read8();
     GPS_waypoint_step=read8();
  }
#endif //MULTIWII_V24

  if (cmdMSP==MSP_ATTITUDE)
  {
    for(uint8_t i=0;i<2;i++){
      MwAngle[i] = MSP_read16();
    }
      MwHeading = MSP_read16();
    #if defined(USEGPSHEADING)
      MwHeading = GPS_ground_course/10;
    #endif
    #ifdef HEADINGCORRECT
      if (MwHeading >= 180) MwHeading -= 360;
    #endif
  }

#if defined DEBUGMW
  if (cmdMSP==MSP_DEBUG)
  {
    for(uint8_t i=0;i<4;i++)
      debug[i] = MSP_read16();
 }
#endif
#ifdef SPORT
  if (cmdMSP==MSP_CELLS)
  {
    for(uint8_t i=0;i<6;i++)
      cell_data[i] = MSP_read16();
  }
#endif //SPORT
  if (cmdMSP==MSP_ALTITUDE)
  {
    #if defined(USEGPSALTITUDE)
      MwAltitude = (int32_t)GPS_altitude*100;
      gpsvario();
    #else    
      MwAltitude =MSP_read32();
      MwVario = MSP_read16();
    #endif
  }

  if (cmdMSP==MSP_ANALOG)
  {
    MwVBat=read8();
    pMeterSum=MSP_read16();
    MwRssi = MSP_read16();
    MWAmperage = MSP_read16();
 }

#ifdef USE_FC_VOLTS_CONFIG
  if (cmdMSP==MSP_MISC)
  {
    MSP_read16(); //ignore: midrc

    MSP_read16(); //ignore: minthrottle
    MSP_read16(); //ignore: maxthrottle
    MSP_read16(); //ignore: mincommand

    MSP_read16(); //ignore: failsafe_throttle
    
    read8(); //ignore: gps_type
    read8(); //ignore: gps_baudrate
    read8(); //ignore: gps_ubx_sbas

    read8(); //ignore: multiwiiCurrentMeterOutput
    read8(); //ignore: rssi_channel
    read8(); //ignore: 0

    MSP_read16(); //ignore: mag_declination

    read8(); //ignore: vbatscale
    MvVBatMinCellVoltage = read8(); //vbatmincellvoltage
    MvVBatMaxCellVoltage = read8(); //vbatmaxcellvoltage
    MvVBatWarningCellVoltage = read8(); //vbatwarningcellvoltage
    
  }
#endif //USE_FC_VOLTS_CONFIG

#if defined (CORRECT_MSP_BF1)  
  if (cmdMSP==MSP_CONFIG)
  {
    for(uint8_t i=0; i<25; i++) {
      bfconfig[i]=read8();
    }
    rollRate = bfconfig[18];
    PitchRate = bfconfig[19];
    modeMSPRequests &=~ REQ_MSP_CONFIG;    
  }
#endif  
  
  if (cmdMSP==MSP_RC_TUNING)
  {
    #ifdef CORRECT_MSP_CF2
      rcRate8 = read8();
      rcExpo8 = read8();
      rollRate = read8();
      PitchRate = read8();
      yawRate = read8();
      dynThrPID = read8();
      thrMid8 = read8();
      thrExpo8 = read8();
      tpa_breakpoint16 = MSP_read16();
      rcYawExpo8 = read8();
      modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #elif defined CORRECT_MSP_CF1
      rcRate8 = read8();
      rcExpo8 = read8();
      rollRate = read8();
      PitchRate = read8();
      yawRate = read8();
      dynThrPID = read8();
      thrMid8 = read8();
      thrExpo8 = read8();
      tpa_breakpoint16 = MSP_read16();
      modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #else
      rcRate8 = read8();
      rcExpo8 = read8();
      rollPitchRate = read8();
      yawRate = read8();
      dynThrPID = read8();
      thrMid8 = read8();
      thrExpo8 = read8();
      modeMSPRequests &=~ REQ_MSP_RC_TUNING;
    #endif
  }

  if (cmdMSP==MSP_PID)
  {
    for(uint8_t i=0; i<PIDITEMS; i++) {
      P16[i] = read8();
      I16[i] = read8();
      D16[i] = read8();
    }
    modeMSPRequests &=~ REQ_MSP_PID;

  }

#ifdef ENABLE_MSP_SAVE_ADVANCED
  if (cmdMSP == MSP_PID_CONTROLLER)
  {
    PIDController = read8();
    modeMSPRequests &=~ REQ_MSP_PID_CONTROLLER;
  }

  if (cmdMSP == MSP_LOOP_TIME)
  {
    LoopTime = MSP_read16();
    modeMSPRequests &=~ REQ_MSP_LOOP_TIME;
  }
#endif

#ifdef HAS_ALARMS
  if (cmdMSP == MSP_ALARMS)
  {
      alarmState = read8();
      alarmMsg[min(dataSize-1, MAX_ALARM_LEN-1)] = 0;
      for(uint8_t i = 0; i < dataSize-1; i++) {
          alarmMsg[min(i, MAX_ALARM_LEN-1)] = read8();
      }
  }
#endif /* HAS_ALARMS */

#ifdef BOXNAMES
  if(cmdMSP==MSP_BOXNAMES) {
    flags.box=1;
    uint32_t bit = 1;
    uint8_t remaining = dataSize;
    uint8_t len = 0;

    mode.armed = 0;
    mode.stable = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.llights = 0;
    mode.camstab = 0;
    mode.osd_switch = 0;
    mode.air = 0;
    mode.acroplus = 0;

    char boxname[20];

    while(remaining > 0) {
      char c = read8();
      if(c != ';') {
        boxname[len] = c;
        len++;
      }
      else {
        if(strncmp("ARM", boxname, len) == 0)
          mode.armed |= bit;
        if(strncmp("ANGLE", boxname, len) == 0)
          mode.stable |= bit;
        if(strncmp("HORIZON", boxname, len) == 0)
          mode.horizon |= bit;
        if(strncmp("MAG", boxname, len) == 0)
          mode.mag |= bit;
        if(strncmp("BARO", boxname, len) == 0)
          mode.baro |= bit;
        if(strncmp("LLIGHTS", boxname, len) == 0)
          mode.llights |= bit;
        if(strncmp("CAMSTAB", boxname, len) == 0)
          mode.camstab |= bit;
        if(strncmp("AIR MODE", boxname, len) == 0)
          mode.air |= bit;
        if(strncmp("ACRO PLUS", boxname, len) == 0)
          mode.acroplus |= bit;
        if(strncmp("GPS HOME", boxname, len) == 0)
          mode.gpshome |= bit;
        if(strncmp("GPS HOLD", boxname, len) == 0)
          mode.gpshold |= bit;
        if(strncmp("PASSTHRU", boxname, len) == 0)
          mode.passthru |= bit;
        if(strncmp("OSD SW", boxname, len) == 0)
          mode.osd_switch |= bit;

        len = 0;
        bit <<= 1L;
      }
      --remaining;
    }
  }
#else  
  if(cmdMSP==MSP_BOXIDS) {
    flags.box=1;
    uint32_t bit = 1;
    uint8_t remaining = dataSize;

    mode.armed = 0;
    mode.stable = 0;
    mode.horizon = 0;
    mode.baro = 0;
    mode.mag = 0;
    mode.gpshome = 0;
    mode.gpshold = 0;
    mode.gpsmission = 0;
    mode.gpsland = 0;
    mode.llights = 0;
    mode.passthru = 0;
    mode.osd_switch = 0;
    mode.camstab = 0;
    mode.air = 0;
    mode.acroplus = 0;

    while(remaining > 0) {
      char c = read8();
      switch(c) {
      case 0:
        mode.armed |= bit;
        break;
      case 1:
        mode.stable |= bit;
        break;
      case 2:
        mode.horizon |= bit;
        break;
      case 3:
        mode.baro |= bit;
        break;
      case 5:
        mode.mag |= bit;
        break;
      case 8:
        mode.camstab |= bit;
       break;
      case 10:
        mode.gpshome |= bit;
        break;
      case 11:
        mode.gpshold |= bit;
        break;
      case 12:
        mode.passthru  |= bit;
        break;
      case 16:
        mode.llights |= bit;
        break;
      case 19:
        mode.osd_switch |= bit;
        break;
      case 20:
        mode.gpsmission |= bit;
        break;
      case 21:
        mode.gpsland |= bit;
        break;
      case 28:
        mode.air |= bit;
        break;
#if defined BETAFLIGHT
      case 29:
        mode.acroplus |= bit;
        break;
#endif //BETAFLIGHT        
      }
      bit <<= 1;
      --remaining;
    }
  }
#endif
#endif // GPSOSD
#endif
}
// End of decoded received commands from MultiWii

void serialKSPCheck()
{
#ifdef KISS_FC
  static uint8_t uav_type = 0;
  static uint8_t esc_num = 0;
  static uint8_t esc_temp[6];
  static uint8_t esc_curr[6];
  static uint8_t esc_volt[6];
  static uint16_t temp_volt;
  static uint16_t temp_amp;
  #ifdef MSPACTIVECHECK
    timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
  #endif
  readIndex = 0;
  
  if (KISS_FC_connected == false) { // first answer is allways Config
    P16[0] = (((serialBuffer[0]<<8) | serialBuffer[1])/10);
    P16[1] = (((serialBuffer[2]<<8) | serialBuffer[3])/10);
    P16[2] = (((serialBuffer[4]<<8) | serialBuffer[5])/10);
    I16[0] = (((serialBuffer[6]<<8) | serialBuffer[7]));
    I16[1] = (((serialBuffer[8]<<8) | serialBuffer[9]));
    I16[2] = (((serialBuffer[10]<<8) | serialBuffer[11]));
    D16[0] = (((serialBuffer[12]<<8) | serialBuffer[13])/10);
    D16[1] = (((serialBuffer[14]<<8) | serialBuffer[15])/10);
    D16[2] = (((serialBuffer[16]<<8) | serialBuffer[17])/10);
    P16[7] = (((serialBuffer[18]<<8) | serialBuffer[19])/10);
    I16[7] = (((serialBuffer[20]<<8) | serialBuffer[21])/10);
    D16[7] = (((serialBuffer[22]<<8) | serialBuffer[23])/10);
    uav_type = serialBuffer[51];                                    // 0:Tri,1:Q+,2:QX,3:Y4,4:Y6,5:H+,6:HX
    if      (uav_type == 0) esc_num = 3;
    else if (uav_type < 4)  esc_num = 4;
    else                    esc_num = 6;
  }
  else {
//    esc_volt[0] = (uint8_t)(((serialBuffer[85]<<8) | serialBuffer[86])/10);
//    esc_volt[1] = (uint8_t)(((serialBuffer[95]<<8) | serialBuffer[96])/10);
//    esc_volt[2] = (uint8_t)(((serialBuffer[105]<<8) | serialBuffer[106])/10);
//    esc_volt[3] = (uint8_t)(((serialBuffer[115]<<8) | serialBuffer[116])/10);
//    esc_volt[4] = (uint8_t)(((serialBuffer[125]<<8) | serialBuffer[126])/10);
//    esc_volt[5] = (uint8_t)(((serialBuffer[135]<<8) | serialBuffer[136])/10);
//    temp_volt = (esc_volt[0]+esc_volt[1]+esc_volt[2]+esc_volt[3]+esc_volt[4]+esc_volt[5])/esc_num;
//    if (temp_volt > 0) MwVBat = temp_volt;
//    else               MwVBat = (uint16_t)(((serialBuffer[17]<<8) | serialBuffer[18])/10);
    MwVBat = (uint16_t)(((serialBuffer[17]<<8) | serialBuffer[18])/10);
    
    esc_curr[0] = (uint8_t)(((serialBuffer[87]<<8) | serialBuffer[88])/13);
    esc_curr[1] = (uint8_t)(((serialBuffer[97]<<8) | serialBuffer[98])/13);
    esc_curr[2] = (uint8_t)(((serialBuffer[107]<<8) | serialBuffer[108])/13);
    esc_curr[3] = (uint8_t)(((serialBuffer[117]<<8) | serialBuffer[118])/13);
    esc_curr[4] = (uint8_t)(((serialBuffer[127]<<8) | serialBuffer[128])/13);
    esc_curr[5] = (uint8_t)(((serialBuffer[137]<<8) | serialBuffer[138])/13);
    temp_amp = (esc_curr[0]+esc_curr[1]+esc_curr[2]+esc_curr[3]+esc_curr[4]+esc_curr[5])*13;
    MWAmperage = temp_amp;
    amperagesum = 360*((serialBuffer[148]<<8) | serialBuffer[149]);
    
    temperature = 0;
    for (uint8_t i = 0; i<6; i++) {
      esc_temp[i] = serialBuffer[84+(i*10)];
      temperature = max(esc_temp[i], temperature);
    }
    MwRcData[THROTTLESTICK] = 1000 + ((serialBuffer[0]<<8) | serialBuffer[1]);
    MwRcData[ROLLSTICK]     = 1500 + (((serialBuffer[2]<<8) | serialBuffer[3])>>1); // -1000 ~1000
    MwRcData[PITCHSTICK]    = 1500 + (((serialBuffer[4]<<8) | serialBuffer[5])>>1);
    MwRcData[YAWSTICK]      = 1500 + (((serialBuffer[6]<<8) | serialBuffer[7])>>1);
    MwRcData[AUX1]          = 1500 + (((serialBuffer[8]<<8) | serialBuffer[9])>>1);
    MwRcData[AUX2]          = 1500 + (((serialBuffer[10]<<8) | serialBuffer[11])>>1);
    MwRcData[AUX3]          = 1500 + (((serialBuffer[12]<<8) | serialBuffer[13])>>1);
    MwRcData[AUX4]          = 1500 + (((serialBuffer[14]<<8) | serialBuffer[15])>>1);
    handleRawRC();
    armed  = serialBuffer[16];
    MwAngle[0] = ((serialBuffer[31]<<8) | serialBuffer[32])/10;
    MwAngle[1] = ((serialBuffer[33]<<8) | serialBuffer[34])/10;
  }
  KISS_FC_connected = true;  
#endif // KISS_FC
}
// --------------------------------------------------------------------------------------

void handleRawRC() {
  static uint8_t waitStick;
  static uint32_t stickTime;
  static uint32_t timeout;

  if(MwRcData[PITCHSTICK] > 1300 && MwRcData[PITCHSTICK] < 1700 &&
     MwRcData[ROLLSTICK] > 1300 && MwRcData[ROLLSTICK] < 1700 &&
     MwRcData[YAWSTICK] > 1300 && MwRcData[YAWSTICK] < 1700) {
	waitStick = 0;
        timeout = 1000;
  }
  else if(waitStick == 1) {
    if((millis() - stickTime) > timeout)
      waitStick = 0;
      timeout = 300;
  }

  if(!waitStick)
  {
    if((MwRcData[PITCHSTICK]>MAXSTICK)&&(MwRcData[YAWSTICK]>MAXSTICK)&&(MwRcData[THROTTLESTICK]>MINSTICK)){
      if (!configMode&&(allSec>5)&&!armed){
          // Enter config mode using stick combination
          waitStick =  2;	// Sticks must return to center before continue!
          configMode = 1;
          setMspRequests();
      }
    }
    else if(configMode) {
      int8_t oldmenudir=constrain(menudir,-5,5);
      menudir=0;
      if(previousarmedstatus&&(MwRcData[THROTTLESTICK]>1300))
      {
	// EXIT from SHOW STATISTICS AFTER DISARM (push throttle up)
	waitStick = 2;
	configExit();
      }
#ifdef MODE1
      if(configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // MOVE RIGHT
#else
      if(configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // MOVE RIGHT
#endif
      {
	waitStick = 1;
	COL++;
	if(COL>3) COL=3;
      }
#ifdef MODE1
      else if(configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // MOVE LEFT
#else
      else if(configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // MOVE LEFT
#endif
      {
	waitStick = 1;
	COL--;
	if(COL<1) COL=1;
      }
      else if(configMode&&(MwRcData[PITCHSTICK]>MAXSTICK)) // MOVE UP
      {
	waitStick = 1;
	ROW--;
	if(ROW<1)
	  ROW=1;
        if(configPage == 0) {
          ROW=10;
        }
      }
      else if(configMode&&(MwRcData[PITCHSTICK]<MINSTICK)) // MOVE DOWN
      {
	waitStick = 1;
	ROW++;
	if(ROW>10)
	  ROW=10;
      }
#ifdef MODE1
      else if(!previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]<MINSTICK)) // DECREASE
#else
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]<MINSTICK)) // DECREASE
#endif
      {
	waitStick = 1;
        menudir=-1+oldmenudir;
        serialMenuCommon();  
      }
#ifdef MODE1
      else if(!previousarmedstatus&&configMode&&(MwRcData[ROLLSTICK]>MAXSTICK)) // INCREASE
#else
      else if(!previousarmedstatus&&configMode&&(MwRcData[YAWSTICK]>MAXSTICK)) // INCREASE
#endif
      { 
	waitStick =1;
        menudir=1+oldmenudir;
        #ifdef MENU9
	if(configPage == MENU9 && COL == 3) {
	  if(ROW==5) timer.magCalibrationTimer=0;
        }
        #endif //MENU9
        serialMenuCommon();  
      }      
    }
    if(waitStick == 1)
      stickTime = millis();
  }
}

void serialMenuCommon()
  {
    if((ROW==10)&&(COL==3)) {
      if (menudir>1){
        menudir=1;
      }
      if (menudir<-1){
        menudir=-1;
      }
//      constrain(menudir,-1,1);
      configPage=configPage+menudir;
    }
    if(configPage<MINPAGE) configPage = MAXPAGE;
    if(configPage>MAXPAGE) configPage = MINPAGE;
#ifdef MENU1
	if(configPage == MENU1) {
	  if(ROW >= 1 && ROW <= 7) {
            uint8_t MODROW=ROW-1;
            if (ROW>5){
              MODROW=ROW+1;
            }
  	    if(COL==1) P16[MODROW]=P16[MODROW]+menudir;
	    if(COL==2) I16[MODROW]=I16[MODROW]+menudir;
	    if(COL==3) D16[MODROW]=D16[MODROW]+menudir;
	  }
	}
#endif
#ifdef MENU2
        #if defined CORRECT_MENU_RCT2
          if(configPage == MENU2 && COL == 3) {
	    if(ROW==1) rcRate8=rcRate8+menudir;
	    if(ROW==2) rcExpo8=rcExpo8+menudir;
	    if(ROW==3) rollRate=rollRate+menudir;
	    if(ROW==4) PitchRate=PitchRate+menudir;
	    if(ROW==5) yawRate=yawRate+menudir;
	    if(ROW==6) dynThrPID=dynThrPID+menudir;
	    if(ROW==7) thrMid8=thrMid8+menudir;
	    if(ROW==8) thrExpo8=thrExpo8+menudir;
	    if(ROW==9) tpa_breakpoint16=tpa_breakpoint16+menudir;
          }
        #elif defined CORRECT_MENU_RCT1
          if(configPage == MENU2 && COL == 3) {
	    if(ROW==1) rcRate8=rcRate8+menudir;
	    if(ROW==2) rcExpo8=rcExpo8+menudir;
	    if(ROW==3) rollRate=rollRate+menudir;
	    if(ROW==4) PitchRate=PitchRate+menudir;
	    if(ROW==5) yawRate=yawRate+menudir;
	    if(ROW==6) dynThrPID=dynThrPID+menudir;
	    if(ROW==7) thrMid8=thrMid8+menudir;
	    if(ROW==8) thrExpo8=thrExpo8+menudir;
         }
        #else
          if(configPage == MENU2 && COL == 3) {
	    if(ROW==1) rcRate8=rcRate8+menudir;
	    if(ROW==2) rcExpo8=rcExpo8+menudir;
	    if(ROW==3) rollPitchRate=rollPitchRate+menudir;
	    if(ROW==4) yawRate=yawRate+menudir;
	    if(ROW==5) dynThrPID=dynThrPID+menudir;
	    if(ROW==6) thrMid8=thrMid8+menudir;
	    if(ROW==7) thrExpo8=thrExpo8+menudir;
	  }
        #endif
#endif
#ifdef MENU3
	if(configPage == MENU3 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAYVOLTAGE]=!Settings[S_DISPLAYVOLTAGE];  
	  if(ROW==2) Settings[S_DIVIDERRATIO]=Settings[S_DIVIDERRATIO]+menudir;
	  if(ROW==3) Settings[S_VOLTAGEMIN]=Settings[S_VOLTAGEMIN]+menudir;
	  if(ROW==4) Settings[S_VIDVOLTAGE]=!Settings[S_VIDVOLTAGE];
	  if(ROW==5) Settings[S_VIDDIVIDERRATIO]=Settings[S_VIDDIVIDERRATIO]+menudir;
	  if(ROW==6) Settings[S_BATCELLS]=Settings[S_BATCELLS]+menudir;
	  if(ROW==7) Settings[S_MAINVOLTAGE_VBAT]=!Settings[S_MAINVOLTAGE_VBAT];
	}
#endif
#ifdef MENU4
	if(configPage == MENU4 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAYRSSI]=!Settings[S_DISPLAYRSSI];
	  if(ROW==2) timer.rssiTimer=15; // 15 secs to turn off tx anwait to read min RSSI
	  if(ROW==3) Settings[S_MWRSSI]=!Settings[S_MWRSSI];
	  if(ROW==4) Settings[S_PWMRSSI]=!Settings[S_PWMRSSI];
	  if(ROW==5) Settings16[S16_RSSIMAX]=Settings16[S16_RSSIMAX]+menudir;
	  if(ROW==6) Settings16[S16_RSSIMIN]=Settings16[S16_RSSIMIN]+menudir;
	}
#endif
#ifdef MENU5
	if(configPage == MENU5 && COL == 3) {
	  if(ROW==1) Settings[S_AMPERAGE]=!Settings[S_AMPERAGE];
	  if(ROW==2) Settings[S_AMPER_HOUR]=!Settings[S_AMPER_HOUR];
	  if(ROW==3) Settings[S_AMPERAGE_VIRTUAL]=!Settings[S_AMPERAGE_VIRTUAL];
	  if(ROW==4) Settings16[S16_AMPDIVIDERRATIO]=Settings16[S16_AMPDIVIDERRATIO]+menudir;
	  if(ROW==5) Settings16[S16_AMPZERO]=Settings16[S16_AMPZERO]+menudir;
	}
#endif
#ifdef MENU6
	if(configPage == MENU6 && COL == 3) {
	  if(ROW==1) Settings[S_DISPLAY_HORIZON_BR]=!Settings[S_DISPLAY_HORIZON_BR];
	  if(ROW==2) Settings[S_WITHDECORATION]=!Settings[S_WITHDECORATION];
	  if(ROW==3) Settings[S_SCROLLING]=!Settings[S_SCROLLING];
	  if(ROW==4) Settings[S_THROTTLEPOSITION]=!Settings[S_THROTTLEPOSITION];
	  if(ROW==5) Settings[S_COORDINATES]=!Settings[S_COORDINATES];
	  if(ROW==6) Settings[S_MODESENSOR]=!Settings[S_MODESENSOR];
	  if(ROW==7) Settings[S_GIMBAL]=!Settings[S_GIMBAL];
	  if(ROW==8) Settings[S_MAPMODE]=Settings[S_MAPMODE]+menudir;
	}
#endif
#ifdef MENU7
	if(configPage == MENU7 && COL == 3) {
	  if(ROW==1) Settings[S_UNITSYSTEM]=!Settings[S_UNITSYSTEM];
	  if(ROW==2) {
	    Settings[S_VIDEOSIGNALTYPE]=!Settings[S_VIDEOSIGNALTYPE];
	    MAX7456Setup();
	    }
	  if(ROW==3) Settings[S_VREFERENCE]=!Settings[S_VREFERENCE];
	  if(ROW==4) Settings[S_DEBUG]=!Settings[S_DEBUG];
	  if(ROW==5) timer.magCalibrationTimer=CALIBRATION_DELAY;
	  if(ROW==6) Settings[S_RCWSWITCH_CH]=Settings[S_RCWSWITCH_CH]+menudir;	}
#endif
#ifdef MENU8
	if(configPage == MENU8 && COL == 3) {
	  if(ROW==1) Settings[S_GPSTIME]=!Settings[S_GPSTIME];
	  if(ROW==2) Settings[S_GPSTZAHEAD]=!Settings[S_GPSTZAHEAD];
	  if(ROW==3) if((menudir == 1 && Settings[S_GPSTZ] < 130) || (menudir == -1 && Settings[S_GPSTZ] > 0))Settings[S_GPSTZ]=Settings[S_GPSTZ]+menudir*5;
	}
#endif
#ifdef MENU9
	if(configPage == MENU9 && COL == 3) {
	  if(ROW==1) Settings[S_DISTANCE_ALARM]=Settings[S_DISTANCE_ALARM]+menudir;
	  if(ROW==2) Settings[S_ALTITUDE_ALARM]=Settings[S_ALTITUDE_ALARM]+menudir;
	  if(ROW==3) Settings[S_SPEED_ALARM]=Settings[S_SPEED_ALARM]+menudir;
	  if(ROW==4) Settings[S_FLYTIME_ALARM]=Settings[S_FLYTIME_ALARM]+menudir;
	  if(ROW==5) Settings[S_AMPER_HOUR_ALARM]=Settings[S_AMPER_HOUR_ALARM]+menudir;
	  if(ROW==6) Settings[S_AMPERAGE_ALARM]=Settings[S_AMPERAGE_ALARM]+menudir;
	}
#endif
#ifdef MENU10
	if(configPage == MENU10 && COL == 3) {
	  if(ROW==1) FCProfile=FCProfile+menudir;
	  if(ROW==2) PIDController=PIDController+menudir;
        #ifdef CORRECTLOOPTIME
	  if(ROW==3) LoopTime=LoopTime+menudir;
        #endif
	};
  #ifdef ENABLE_MSP_SAVE_ADVANCED
        if (FCProfile>2)
          FCProfile=0;
        if (FCProfile!=PreviousFCProfile){
          setFCProfile();
          PreviousFCProfile=FCProfile;
        }        
  #endif
#endif  
	if((ROW==10)&&(COL==1)) configExit();
	if((ROW==10)&&(COL==2)) configSave();
}

void serialReceive(uint8_t loops)
{
  uint8_t c;
  uint8_t loopserial=0;

  static uint8_t prot_type = 0;
  static enum serial_states {
    IDLE,
    MSP_HEADER,
    KSP_HEADER,
    HEADER_M,
    HEADER_ARROW,
    MSP_HEADER_SIZE,
    KSP_HEADER_SIZE,
    HEADER_CMD
  } c_state = IDLE;

  if (Serial.available()) loopserial=1;
  while(loopserial==1)
  {
    c = Serial.read();

    #ifdef GPSOSD    
      armedtimer = 0;
      #if defined (NAZA)
        NAZA_NewData(c);
      #else
        if (GPS_newFrame(c)) GPS_NewData();  
      #endif //NAZA  
    #endif //GPSOSD 

    if (c_state == IDLE)
    {
      if (c=='$') {
        prot_type = MSP_HEADER;
        c_state = MSP_HEADER;
      }
      else if (c==5) {
        prot_type = KSP_HEADER;
        c_state = KSP_HEADER;
      }
      else c_state = IDLE;
    }
    else if (c_state == MSP_HEADER)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if ((c_state == HEADER_ARROW) || (c_state == KSP_HEADER)) 
    {
      if (c > SERIALBUFFERSIZE)
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        dataSize = c;
        c_state = (prot_type == MSP_HEADER) ? MSP_HEADER_SIZE : KSP_HEADER_SIZE;
        MSP_rcvChecksum =  c;
        KSP_rcvChecksum =  0;
        receiverIndex=0;
      }
    }
    else if (c_state == MSP_HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      MSP_rcvChecksum ^= c;
    }
    else if (c_state == HEADER_CMD)
    {
      MSP_rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(MSP_rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
    else if (c_state == KSP_HEADER_SIZE)
    {
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(KSP_rcvChecksum/dataSize == c) {
            serialKSPCheck();
        }
        c_state = IDLE;
      }
      else
        KSP_rcvChecksum += c;
        serialBuffer[receiverIndex++]=c;
    }
    
    if (loops==0) loopserial=0;
    if (!Serial.available()) loopserial=0;
  }
}

void configExit()
{
  configPage=1;
  ROW=10;
  COL=3;
  configMode=0;
  //waitStick=3;
  previousarmedstatus = 0;
  if (Settings[S_RESETSTATISTICS]){
    trip=0;
    distanceMAX=0;
    altitudeMAX=0;
    speedMAX=0;
    ampMAX=0;
    flyingTime=0;
    temperatureMAX=0;
  }
  #ifdef ENABLE_MSP_SAVE_ADVANCED
    if (FCProfile!=CurrentFCProfile){
      FCProfile=CurrentFCProfile;
      setFCProfile();
    }
  #endif
  setMspRequests();
}

void configSave()
{
  CurrentFCProfile=FCProfile;

#if defined ENABLE_MSP_SAVE_ADVANCED
  WriteGerneralRequest(MSP_SET_PID_CONTROLLER, 1);
  mspWrite8(PIDController);
  mspWriteChecksum();

  WriteGerneralRequest(MSP_SET_LOOP_TIME, 2);
  mspWrite16(LoopTime);
  mspWriteChecksum();  
#endif

  WriteGerneralRequest(MSP_SET_PID, PIDITEMS*3);
  for(uint8_t i=0; i<PIDITEMS; i++) {
    mspWrite8(P16[i]);
    mspWrite8(I16[i]);
    mspWrite8(D16[i]);
  }
  mspWriteChecksum();
  
#if defined CORRECT_MSP_CF2
  WriteGerneralRequest(MSP_SET_RC_TUNING,11);
  mspWrite8(rcRate8);
  mspWrite8(rcExpo8);
  mspWrite8(rollRate);
  mspWrite8(PitchRate);
  mspWrite8(yawRate);
  mspWrite8(dynThrPID);
  mspWrite8(thrMid8);
  mspWrite8(thrExpo8);
  mspWrite16(tpa_breakpoint16);
  mspWrite8(rcYawExpo8);
  mspWriteChecksum();
#elif defined CORRECT_MSP_CF1
  WriteGerneralRequest(MSP_SET_RC_TUNING,10);
  mspWrite8(rcRate8);
  mspWrite8(rcExpo8);
  mspWrite8(rollRate);
  mspWrite8(PitchRate);
  mspWrite8(yawRate);
  mspWrite8(dynThrPID);
  mspWrite8(thrMid8);
  mspWrite8(thrExpo8);
  mspWrite16(tpa_breakpoint16);
  mspWriteChecksum();
#else
  WriteGerneralRequest(MSP_SET_RC_TUNING,7);
  mspWrite8(rcRate8);
  mspWrite8(rcExpo8);
  mspWrite8(rollPitchRate);
  mspWrite8(yawRate);
  mspWrite8(dynThrPID);
  mspWrite8(thrMid8);
  mspWrite8(thrExpo8);
  mspWriteChecksum();
 #endif

#if defined CORRECT_MSP_BF1
  WriteGerneralRequest(MSP_SET_CONFIG,25);
  bfconfig[18] =rollRate;
  bfconfig[19] =PitchRate;
  for(uint8_t i=0; i<25; i++) {
    mspWrite8(bfconfig[i]);
  }
  mspWriteChecksum();
#endif

  writeEEPROM();
  WriteGerneralRequest(MSP_EEPROM_WRITE,0);
  configExit();
}

void fontSerialRequest() {
  WriteGerneralRequest(MSP_OSD,3);
  mspWrite8(OSD_GET_FONT);
  mspWrite16(getNextCharToRequest());
  mspWriteChecksum();
}

void settingsSerialRequest() {
  WriteGerneralRequest(MSP_OSD,1+30);
  mspWrite8(OSD_READ_CMD_EE);
  for(uint8_t i=0; i<10; i++) {
    eedata = EEPROM.read(eeaddress);
    mspWrite16(eeaddress);
    mspWrite8(eedata);
    eeaddress++;
  }
  mspWriteChecksum();
}

void settingswriteSerialRequest() {
  WriteGerneralRequest(MSP_OSD,3);
  mspWrite8(OSD_READ_CMD_EE);
  mspWrite16(eeaddress);
  mspWriteChecksum();
}

void setFCProfile()
{
  WriteGerneralRequest(MSP_SELECT_SETTING, 1);
  mspWrite8(FCProfile);
  mspWriteChecksum();
  WriteGerneralRequest(MSP_EEPROM_WRITE, 0);
  setMspRequests();
  delay(100);
}
