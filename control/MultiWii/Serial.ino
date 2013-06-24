
void SerialCom() {
  writeStatusDataToSerial();
  if( SerialAvailable() ) {
    readCommandsFromSerial();
  }
}

//Output format: time;acc0,acc1,acc2;gyro0,gyro1,gyro2;mag0,mag1,mag2;alt;vario\n
//writes current status to serial port
void writeStatusDataToSerial()
{
  Serial.print( millis() );
//  Serial.print( " ACC " );
  Serial.print( ';' );
  for( uint8_t i = 0; i < 3; i++ ) {
    Serial.print( accSmooth[i] );
    Serial.print( ',' );
  }
//  Serial.print( "GYRO " );
  Serial.print( ';' );
  for( uint8_t i = 0; i < 3; i++ ) {
    Serial.print( gyroData[i] );
    Serial.print( ',' );
  }
//  Serial.print( "MAG " );
  Serial.print( ';' );
  for( uint8_t i = 0; i < 3; i++ ) {
    Serial.print( magADC[i] );
    Serial.print( ',' );
  }
//  Serial.print( "EST_ALT " );
  Serial.print( ';' );
  Serial.print( EstAlt );
//  Serial.print( " VARIO " );
  Serial.print( ';' );
//  Serial.print( vario );
//  Serial.println( " END" );
  Serial.println( vario );
}

//input format: M<handle value
//reads comands from serial port to global variables
void readCommandsFromSerial()
{
  Serial.find( "M<" );
  //command format: control_handle value
  int control_handle = Serial.parseInt();
  if( Serial.read() != ' ' ) {
//    Serial.println( "BAD1" );
    return;
  }
  int value = Serial.parseInt();
  if( Serial.read() != '\n' ) {
//    Serial.println( "BAD2" );
    return;
  }
  //просто задаём команды rc
  rcData[control_handle] = value;
//  Serial.println( "OK" );
}
 
void SerialOpen( uint32_t baud ) {
  Serial.begin( baud );
}
  
uint8_t SerialRead() {
  return Serial.read();
}
  
uint8_t SerialAvailable() {
  return Serial.available();
}
  
void SerialWrite( uint8_t c )
{
  Serial.write( c );
}

/*
#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module. 
// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t PROGMEM capability = 0+BIND_CAPABLE;

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE]; //buffer for command body
static uint8_t checksum; //current checksum (for current command)
static uint8_t indRX;  //reading index inside buffer inBuf
static uint8_t cmdMSP; //current read command id

uint32_t readFromInBuf32() 
{
  uint32_t t = readFromInBuf16();
  t += (uint32_t)readFromInBuf16() << 16;
  return t;
}

uint16_t readFromInBuf16() 
{
  uint16_t t = readFromInBuf8();
  t += (uint16_t)readFromInBuf8() << 8;
  return t;
}

uint8_t readFromInBuf8() 
{
  return inBuf[indRX++] & 0xff;
}

//comment out to use standart Serial object for communication
#define USE_MULTIWII_SERIAL_CODE

#ifdef USE_MULTIWII_SERIAL_CODE
  // *******************************************************
  // Interrupt driven UART transmitter - using a ring buffer
  // *******************************************************
  
  #define RX_BUFFER_SIZE 64
  #define TX_BUFFER_SIZE 128
  
  //input ring buffer and head|tail pointers
  static volatile uint8_t serialHeadRX, serialTailRX;
  static uint8_t serialBufferRX[RX_BUFFER_SIZE];
  //output ring buffer and head|tail pointers
  static volatile uint8_t serialHeadTX, serialTailTX;
  static uint8_t serialBufferTX[TX_BUFFER_SIZE];
  
  //interrupt handler for storing recieved byte into serialBufferRX
  ISR(USART_RX_vect) { 
    uint8_t h = serialHeadRX;
    if( ++h >= RX_BUFFER_SIZE ) {
      h = 0;
    }
    if( h == serialTailRX ) {
      return; // we did not bite our own tail?
    }
    serialBufferRX[serialHeadRX] = UDR0;  
    serialHeadRX = h;
  }
  
  //interrupt handler for transmission-ready interrupt - sends next byte from ring buffer
  ISR(USART_UDRE_vect) {
      uint8_t t = serialTailTX;
      if (serialHeadTX != t) {
        if (++t >= TX_BUFFER_SIZE) {
          t = 0;
        }
        UDR0 = serialBufferTX[t];  // Transmit next byte in the ring
        serialTailTX = t;
      }
      if (t == serialHeadTX) {
        UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
      }
  }
  
  void writeToRingBuffer32(uint32_t a) {
    writeToRingBuffer8((a    ) & 0xFF);
    writeToRingBuffer8((a>> 8) & 0xFF);
    writeToRingBuffer8((a>>16) & 0xFF);
    writeToRingBuffer8((a>>24) & 0xFF);
  }
  
  void writeToRingBuffer16(int16_t a) {
    writeToRingBuffer8((a   ) & 0xFF);
    writeToRingBuffer8((a>>8) & 0xFF);
  }
  
  void writeToRingBuffer8(uint8_t a) {
    uint8_t t = serialHeadTX;
    if( ++t >= TX_BUFFER_SIZE ) {
      t = 0;
    }
    serialBufferTX[t] = a;
    checksum ^= a;
    serialHeadTX = t;
  }
  
  void SerialOpen( uint32_t baud ) {
    uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
    uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
    //!!!!!!!!Lots of magic !!!!!!!
    UCSR0A  = (1<<U2X0); 
    UBRR0H = h; 
    UBRR0L = l; 
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
  }
  
  uint8_t SerialRead() {
    uint8_t t = serialTailRX;
    uint8_t c = serialBufferRX[t];
    if( serialHeadRX != t ) {
      if( ++t >= RX_BUFFER_SIZE ) {
        t = 0;
      }
      serialTailRX = t;
    }
    return c;
  }
  
  uint8_t SerialAvailable() {
    return ( serialHeadRX - serialTailRX ) % RX_BUFFER_SIZE;
  }
  
  void SerialWrite( uint8_t c )
  {
    writeToRingBuffer8(c);
    UartSendData();
  }
#else //!USE_MULTIWII_SERIAL_CODE

  void writeToRingBuffer32(uint32_t a) {
    writeToRingBuffer8((a    ) & 0xFF);
    writeToRingBuffer8((a>> 8) & 0xFF);
    writeToRingBuffer8((a>>16) & 0xFF);
    writeToRingBuffer8((a>>24) & 0xFF);
  }
  
  void writeToRingBuffer16(int16_t a) {
    writeToRingBuffer8((a   ) & 0xFF);
    writeToRingBuffer8((a>>8) & 0xFF);
  }
  
  void writeToRingBuffer8(uint8_t a) {
    Serial.write( a );
  }
  
  void SerialOpen( uint32_t baud ) {
    Serial.begin( baud );
  }
  
  uint8_t SerialRead() {
  return Serial.read();
  }
  
  uint8_t SerialAvailable() {
    return Serial.available();
  }
  
  void SerialWrite( uint8_t c )
  {
    writeToRingBuffer8(c);
  }
#endif //!USE_MULTIWII_SERIAL_CODE

//start writing response or error (if err != 0) of size s with current command cmdMSP
void headSerialResponse(uint8_t err, uint8_t s) 
{
  writeToRingBuffer8('$');
  writeToRingBuffer8('M');
  writeToRingBuffer8(err ? '!' : '>');
  checksum = 0; // start calculating a new checksum
  writeToRingBuffer8(s);
  writeToRingBuffer8( cmdMSP );
}

//start writing reply of size s
void headSerialReply(uint8_t s) 
{
  headSerialResponse(0, s);
}

//start writing error of size s
void inline headSerialError(uint8_t s) 
{
  headSerialResponse(1, s);
}

//end writing reply
void tailSerialReply() 
{
  writeToRingBuffer8( checksum );
  UartSendData();
}

//transmit all we have written
void UartSendData() {
    UCSR0B |= (1 << UDRIE0); //magic!!!!
}

//serialize 0-ended string from memory address s to serial port
void serializeNames(PGM_P s) 
{
  for( PGM_P c = s; pgm_read_byte(c); c++ ) {
    writeToRingBuffer8(pgm_read_byte(c));
  }
}

//check commands from seral port and respond
void SerialCom() {
  uint8_t c,n;  
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {  //prev read state
    IDLE,  //waiting for next command
    HEADER_START, //read $
    HEADER_M, //read M
    HEADER_ARROW, //read <
    HEADER_SIZE, //read command size
    HEADER_CMD, //read command id
  } c_state;// = IDLE;

  while( SerialAvailable() ) {
 #ifdef USE_MULTIWII_SERIAL_CODE
      uint8_t bytesTXBuff = ( (uint8_t)(serialHeadTX - serialTailTX) ) % TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) {
        return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      }
 #endif
      c = SerialRead();
        // regular data handling to detect and handle MSP and other data
        if (c_state == IDLE) {
          c_state = (c=='$') ? HEADER_START : IDLE;
          //we will not evaluate anything not in our protocol
        } else if( c_state == HEADER_START ) {
          c_state = (c=='M') ? HEADER_M : IDLE;
        } else if( c_state == HEADER_M ) {
          c_state = (c=='<') ? HEADER_ARROW : IDLE;
        } else if( c_state == HEADER_ARROW ) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            c_state = IDLE;
            continue;
          }
          dataSize = c;
          offset = 0;
          checksum = 0;
          indRX = 0;
          checksum ^= c;
          c_state = HEADER_SIZE;  // the command is to follow
        } else if( c_state == HEADER_SIZE ) {
          cmdMSP = c;
          checksum ^= c;
          c_state = HEADER_CMD;
        } else if( c_state == HEADER_CMD ) {
          if( offset < dataSize ) {
            checksum ^= c;
            inBuf[offset++] = c;
          } else { //offset >= dataSize
            if( checksum == c ) {  // compare calculated and transferred checksum
              evaluateCommand();  // we got a valid packet, evaluate it
            }
            c_state = IDLE;
          }
        }
  }
}

//process command for current port
static void inline evaluateCommand() {
  switch( cmdMSP ) {
   case MSP_SET_RAW_RC:
     for( uint8_t i = 0; i < 8; i++ ) {
       rcData[i] = readFromInBuf16();
     }
     headSerialReply(0);
     break;
   #if GPS
   case MSP_SET_RAW_GPS:
     f.GPS_FIX = readFromInBuf8();
     GPS_numSat = readFromInBuf8();
     GPS_coord[LAT] = readFromInBuf32();
     GPS_coord[LON] = readFromInBuf32();
     GPS_altitude = readFromInBuf16();
     GPS_speed = readFromInBuf16();
     GPS_update |= 2;              // New data signalisation to GPS functions
     headSerialReply(0);
     break;
   #endif
   case MSP_SET_PID:
     for( uint8_t i = 0; i < PIDITEMS; i++ ) {
       conf.P8[i]=readFromInBuf8();
       conf.I8[i]=readFromInBuf8();
       conf.D8[i]=readFromInBuf8();
     }
     headSerialReply(0);
     break;
   case MSP_SET_BOX:
     for( uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
       conf.activate[i]=readFromInBuf16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = readFromInBuf8();
     conf.rcExpo8 = readFromInBuf8();
     conf.rollPitchRate = readFromInBuf8();
     conf.yawRate = readFromInBuf8();
     conf.dynThrPID = readFromInBuf8();
     conf.thrMid8 = readFromInBuf8();
     conf.thrExpo8 = readFromInBuf8();
     headSerialReply(0);
     break;
   case MSP_SET_MISC:
     #if defined(POWERMETER)
       conf.powerTrigger1 = readFromInBuf16() / PLEVELSCALE;
     #endif
     headSerialReply(0);
     break;
   case MSP_SET_HEAD:
     magHold = readFromInBuf16();
     headSerialReply(0);
     break;
   case MSP_IDENT:
     headSerialReply(7);
     writeToRingBuffer8(VERSION);   // multiwii version
     writeToRingBuffer8(MULTITYPE); // type of multicopter
     writeToRingBuffer8(MSP_VERSION);         // MultiWii Serial Protocol Version
     writeToRingBuffer32( pgm_read_dword( &(capability) ) );        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(11);
     writeToRingBuffer16(cycleTime);
     writeToRingBuffer16(i2c_errors_count);
     writeToRingBuffer16( ACC | BARO << 1 | MAG << 2 | GPS << 3 | SONAR << 4 );
     writeToRingBuffer32(
                 #if ACC
                   f.ANGLE_MODE << BOXANGLE |
                   f.HORIZON_MODE << BOXHORIZON |
                 #endif
                 #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
                   f.BARO_MODE << BOXBARO |
                 #endif
                 #if MAG
                   f.MAG_MODE << BOXMAG | f.HEADFREE_MODE << BOXHEADFREE | rcOptions[BOXHEADADJ] << BOXHEADADJ |
                 #endif
                 #if GPS
                   f.GPS_HOME_MODE << BOXGPSHOME | f.GPS_HOLD_MODE << BOXGPSHOLD |
                 #endif
                 #if defined(BUZZER)
                   rcOptions[BOXBEEPERON] << BOXBEEPERON |
                 #endif
                 #if defined(VARIOMETER)
                   rcOptions[BOXVARIO] << BOXVARIO |
                 #endif
                 #if defined(INFLIGHT_ACC_CALIBRATION)
                   rcOptions[BOXCALIB] << BOXCALIB |
                 #endif
                 #if defined(GOVERNOR_P)
                   rcOptions[BOXGOV] << BOXGOV |
                 #endif
                 f.ARMED << BOXARM );
       writeToRingBuffer8( global_conf.currentSet );   // current setting
     break;
   case MSP_RAW_IMU:
     headSerialReply(18);
     for( uint8_t i = 0; i < 3; i++ ) {
       writeToRingBuffer16(accSmooth[i]);
     }
     for( uint8_t i = 0; i < 3; i++ ) {
       writeToRingBuffer16(gyroData[i]);
     }
     for( uint8_t i = 0; i < 3; i++ ) {
       writeToRingBuffer16(magADC[i]);
     }
     break;
   case MSP_SERVO:
     headSerialReply(16);
     for( uint8_t i = 0; i < 8; i++ ) {
       writeToRingBuffer16(0);
     }
     break;
   case MSP_MOTOR:
     headSerialReply(16);
     for( uint8_t i = 0; i < 8; i++ ) {
       writeToRingBuffer16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(RC_CHANS * 2);
     for( uint8_t i = 0; i < RC_CHANS; i++ ) {
       writeToRingBuffer16(rcData[i]);
     }
     break;
   #if GPS
   case MSP_RAW_GPS:
     headSerialReply(16);
     writeToRingBuffer8(f.GPS_FIX);
     writeToRingBuffer8(GPS_numSat);
     writeToRingBuffer32(GPS_coord[LAT]);
     writeToRingBuffer32(GPS_coord[LON]);
     writeToRingBuffer16(GPS_altitude);
     writeToRingBuffer16(GPS_speed);
     writeToRingBuffer16(GPS_ground_course);        // added since r1172
     break;
   case MSP_COMP_GPS:
     headSerialReply(5);
     writeToRingBuffer16(GPS_distanceToHome);
     writeToRingBuffer16(GPS_directionToHome);
     writeToRingBuffer8(GPS_update & 1);
     break;
   #endif
   case MSP_ATTITUDE:
     headSerialReply(8);
     for( uint8_t i = 0; i < 2; i++ ) {
       writeToRingBuffer16(angle[i]);
     }
     writeToRingBuffer16(heading);
     writeToRingBuffer16(headFreeModeHold);
     break;
   case MSP_ALTITUDE:
     headSerialReply(6);
     writeToRingBuffer32(EstAlt);
     writeToRingBuffer16(vario);                  // added since r1172
     break;
   case MSP_ANALOG:
     headSerialReply(5);
     writeToRingBuffer8(vbat);
     writeToRingBuffer16(intPowerMeterSum);
     writeToRingBuffer16(rssi);
     break;
   case MSP_RC_TUNING:
     headSerialReply(7);
     writeToRingBuffer8(conf.rcRate8);
     writeToRingBuffer8(conf.rcExpo8);
     writeToRingBuffer8(conf.rollPitchRate);
     writeToRingBuffer8(conf.yawRate);
     writeToRingBuffer8(conf.dynThrPID);
     writeToRingBuffer8(conf.thrMid8);
     writeToRingBuffer8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for( uint8_t i = 0; i < PIDITEMS; i++ ) {
       writeToRingBuffer8(conf.P8[i]);
       writeToRingBuffer8(conf.I8[i]);
       writeToRingBuffer8(conf.D8[i]);
     }
     break;
   case MSP_PIDNAMES:
     headSerialReply(strlen_P(pidnames));
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for( uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
       writeToRingBuffer16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
     headSerialReply(strlen_P(boxnames));
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
     for( uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
       writeToRingBuffer8(pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MISC:
     headSerialReply(2);
     writeToRingBuffer16(intPowerTrigger1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for( uint8_t i = 0; i < 8; i++ ) {
       writeToRingBuffer8(PWM_PIN[i]);
     }
     break;
   case MSP_RESET_CONF:
     if( !f.ARMED ) {
       LoadDefaults();
     }
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     if( !f.ARMED ) {
       calibratingA = 512;
     }
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     if( !f.ARMED ) {
       f.CALIBRATE_MAG = 1;
     }
     headSerialReply(0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     headSerialReply(8);
     for( uint8_t i = 0; i < 4; i++ ) {
       writeToRingBuffer16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}*/
