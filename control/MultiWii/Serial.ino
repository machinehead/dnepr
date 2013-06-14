#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX, serialTailRX;
static uint8_t serialBufferRX[RX_BUFFER_SIZE];
static volatile uint8_t serialHeadTX, serialTailTX;
static uint8_t serialBufferTX[TX_BUFFER_SIZE];
static uint8_t inBuf[INBUF_SIZE];

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

static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

uint32_t read32() 
{
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}

uint16_t read16() 
{
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}

uint8_t read8() 
{
  return inBuf[indRX++] & 0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) 
{
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8( cmdMSP );
}

void headSerialReply(uint8_t s) 
{
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) 
{
  headSerialResponse(1, s);
}

void tailSerialReply() 
{
  serialize8( checksum );
  UartSendData();
}

void serializeNames(PGM_P s) 
{
  for( PGM_P c = s; pgm_read_byte(c); c++ ) {
    serialize8(pgm_read_byte(c));
  }
}

//check commands from seral port and respond
void SerialCom() {
  uint8_t c,n;  
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state;// = IDLE;

  {
    while( SerialAvailable() ) {
      uint8_t bytesTXBuff = ( (uint8_t)(serialHeadTX - serialTailTX) ) % TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) {
        return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      }
      c = SerialRead();
        // regular data handling to detect and handle MSP and other data
        if (c_state == IDLE) {
          c_state = (c=='$') ? HEADER_START : IDLE;
          if (c_state == IDLE) {
  //          evaluateOtherData(c); // evaluate all other incoming serial data
          }
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
        } else if( c_state == HEADER_CMD && offset < dataSize ) {
          checksum ^= c;
          inBuf[offset++] = c;
        } else if( c_state == HEADER_CMD && offset >= dataSize ) {
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
       rcData[i] = read16();
     }
     headSerialReply(0);
     break;
   #if GPS
   case MSP_SET_RAW_GPS:
     f.GPS_FIX = read8();
     GPS_numSat = read8();
     GPS_coord[LAT] = read32();
     GPS_coord[LON] = read32();
     GPS_altitude = read16();
     GPS_speed = read16();
     GPS_update |= 2;              // New data signalisation to GPS functions
     headSerialReply(0);
     break;
   #endif
   case MSP_SET_PID:
     for( uint8_t i = 0; i < PIDITEMS; i++ ) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(0);
     break;
   case MSP_SET_BOX:
     for( uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
       conf.activate[i]=read16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(0);
     break;
   case MSP_SET_MISC:
     #if defined(POWERMETER)
       conf.powerTrigger1 = read16() / PLEVELSCALE;
     #endif
     headSerialReply(0);
     break;
   case MSP_SET_HEAD:
     magHold = read16();
     headSerialReply(0);
     break;
   case MSP_IDENT:
     headSerialReply(7);
     serialize8(VERSION);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32( pgm_read_dword( &(capability) ) );        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(11);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16( ACC | BARO << 1 | MAG << 2 | GPS << 3 | SONAR << 4 );
     serialize32(
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
       serialize8( global_conf.currentSet );   // current setting
     break;
   case MSP_RAW_IMU:
     headSerialReply(18);
     for( uint8_t i = 0; i < 3; i++ ) {
       serialize16(accSmooth[i]);
     }
     for( uint8_t i = 0; i < 3; i++ ) {
       serialize16(gyroData[i]);
     }
     for( uint8_t i = 0; i < 3; i++ ) {
       serialize16(magADC[i]);
     }
     break;
   case MSP_SERVO:
     headSerialReply(16);
     for( uint8_t i = 0; i < 8; i++ ) {
       serialize16(0);
     }
     break;
   case MSP_MOTOR:
     headSerialReply(16);
     for( uint8_t i = 0; i < 8; i++ ) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(RC_CHANS * 2);
     for( uint8_t i = 0; i < RC_CHANS; i++ ) {
       serialize16(rcData[i]);
     }
     break;
   #if GPS
   case MSP_RAW_GPS:
     headSerialReply(16);
     serialize8(f.GPS_FIX);
     serialize8(GPS_numSat);
     serialize32(GPS_coord[LAT]);
     serialize32(GPS_coord[LON]);
     serialize16(GPS_altitude);
     serialize16(GPS_speed);
     serialize16(GPS_ground_course);        // added since r1172
     break;
   case MSP_COMP_GPS:
     headSerialReply(5);
     serialize16(GPS_distanceToHome);
     serialize16(GPS_directionToHome);
     serialize8(GPS_update & 1);
     break;
   #endif
   case MSP_ATTITUDE:
     headSerialReply(8);
     for( uint8_t i = 0; i < 2; i++ ) {
       serialize16(angle[i]);
     }
     serialize16(heading);
     serialize16(headFreeModeHold);
     break;
   case MSP_ALTITUDE:
     headSerialReply(6);
     serialize32(EstAlt);
     serialize16(vario);                  // added since r1172
     break;
   case MSP_ANALOG:
     headSerialReply(5);
     serialize8(vbat);
     serialize16(intPowerMeterSum);
     serialize16(rssi);
     break;
   case MSP_RC_TUNING:
     headSerialReply(7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for( uint8_t i = 0; i < PIDITEMS; i++ ) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
   case MSP_PIDNAMES:
     headSerialReply(strlen_P(pidnames));
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for( uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
       serialize16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
     headSerialReply(strlen_P(boxnames));
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
     for( uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
       serialize8(pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MISC:
     headSerialReply(2);
     serialize16(intPowerTrigger1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for( uint8_t i = 0; i < 8; i++ ) {
       serialize8(PWM_PIN[i]);
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
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX;
  if( ++t >= TX_BUFFER_SIZE ) {
    t = 0;
  }
  serialBufferTX[t] = a;
  checksum ^= a;
  serialHeadTX = t;
}

void UartSendData() {
    UCSR0B |= (1 << UDRIE0);
}


void SerialOpen( uint32_t baud ) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
    
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
  serialize8(c);
  UartSendData();
}
