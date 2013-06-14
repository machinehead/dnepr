// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif


uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

#if defined MMGYRO       
  // Moving Average Gyros by Magnetron1
  //---------------------------------------------------
  static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGTH];
  static int32_t mediaMobileGyroADCSum[3];
  static uint8_t mediaMobileGyroIDX;
  //---------------------------------------------------
#endif

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;
        
        #if defined(GYROCALIBRATIONFAILSAFE)
            previousGyroADC[axis] = gyroADC[axis];
          }
          if (calibratingG % 10 == 0) {
            if(abs(gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
            previousGyroADC[axis] = gyroADC[axis];
       #endif
      }
      // Sum up 512 readings
      g[axis] +=gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]>>9;
      #if defined(BUZZER)
        alarmArray[7] = 4;
      #else
        blinkLED(10,15,1); //the delay causes to beep the buzzer really long 
      #endif
      }
    }
    #if defined(GYROCALIBRATIONFAILSAFE)
      if(tilt) {
        calibratingG=1000;
        LEDPIN_ON;
      } else {
        calibratingG--;
        LEDPIN_OFF;
      }
      return;
    #else
      calibratingG--;
    #endif
    
  }

#ifdef MMGYRO       
  mediaMobileGyroIDX = ++mediaMobileGyroIDX % conf.mmgyro;
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    //anti gyro glitch, limit the variation between two consecutive readings
    mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    gyroADC[axis] = mediaMobileGyroADCSum[axis] / conf.mmgyro;
#else 
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif    
    previousGyroADC[axis] = gyroADC[axis];
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp  = ((gyroADC[PITCH] - gyroADC[ROLL] )*7)/10;
    gyroADC[ROLL] = ((gyroADC[ROLL]  + gyroADC[PITCH])*7)/10;
    gyroADC[PITCH]= temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp  = ((gyroADC[PITCH] + gyroADC[ROLL] )*7)/10;
    gyroADC[ROLL] = ((gyroADC[ROLL]  - gyroADC[PITCH])*7)/10;
    gyroADC[PITCH]= temp;
  #endif
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=accADC[axis];
      // Clear global variables for next reading
      accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = a[ROLL]>>9;
      global_conf.accZero[PITCH] = a[PITCH]>>9;
      global_conf.accZero[YAW]   = (a[YAW]>>9)-acc_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  #if defined(INFLIGHT_ACC_CALIBRATION)
      static int32_t b[3];
      static int16_t accZero_saved[3]  = {0,0,0};
      static int16_t  angleTrim_saved[2] = {0, 0};
      //Saving old zeropoints before measurement
      if (InflightcalibratingA==50) {
         accZero_saved[ROLL]  = global_conf.accZero[ROLL] ;
         accZero_saved[PITCH] = global_conf.accZero[PITCH];
         accZero_saved[YAW]   = global_conf.accZero[YAW] ;
         angleTrim_saved[ROLL]  = conf.angleTrim[ROLL] ;
         angleTrim_saved[PITCH] = conf.angleTrim[PITCH] ;
      }
      if (InflightcalibratingA>0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
          // Reset a[axis] at start of calibration
          if (InflightcalibratingA == 50) b[axis]=0;
          // Sum up 50 readings
          b[axis] +=accADC[axis];
          // Clear global variables for next reading
          accADC[axis]=0;
          global_conf.accZero[axis]=0;
        }
        //all values are measured
        if (InflightcalibratingA == 1) {
          AccInflightCalibrationActive = 0;
          AccInflightCalibrationMeasurementDone = 1;
          #if defined(BUZZER)
            alarmArray[7] = 1;      //buzzer for indicatiing the end of calibration
          #endif
          // recover saved values to maintain current flight behavior until new values are transferred
          global_conf.accZero[ROLL]  = accZero_saved[ROLL] ;
          global_conf.accZero[PITCH] = accZero_saved[PITCH];
          global_conf.accZero[YAW]   = accZero_saved[YAW] ;
          conf.angleTrim[ROLL]  = angleTrim_saved[ROLL] ;
          conf.angleTrim[PITCH] = angleTrim_saved[PITCH] ;
        }
        InflightcalibratingA--;
      }
      // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
      if (AccInflightCalibrationSavetoEEProm == 1){  //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        global_conf.accZero[ROLL]  = b[ROLL]/50;
        global_conf.accZero[PITCH] = b[PITCH]/50;
        global_conf.accZero[YAW]   = b[YAW]/50-acc_1G; // for nunchuk 200=1G
        conf.angleTrim[ROLL]   = 0;
        conf.angleTrim[PITCH]  = 0;
        writeGlobalSet(1); // write accZero in EEPROM
      }
  #endif
  accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  accADC[PITCH] -=  global_conf.accZero[PITCH];
  accADC[YAW]   -=  global_conf.accZero[YAW] ;

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((accADC[PITCH] - accADC[ROLL] )*7)/10;
    accADC[ROLL] = ((accADC[ROLL]  + accADC[PITCH])*7)/10;
    accADC[PITCH] = temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((accADC[PITCH] + accADC[ROLL] )*7)/10;
    accADC[ROLL] = ((accADC[ROLL]  - accADC[PITCH])*7)/10;
    accADC[PITCH] = temp;
  #endif
}

// ************************************************************************************************************
// ************************************************************************************************************
// I2C Accelerometer LSM303DLHC
// ************************************************************************************************************
#if defined(LSM303DLHC_ACC)
#define LSM303DLHC_ACC_ADDRESS 0x19
void ACC_init () {
  delay(10);
  i2c_writeReg(LSM303DLHC_ACC_ADDRESS,0x20,0x77); // REG1_A, normal power mode, 50 Hz, all axis enable
  i2c_writeReg(LSM303DLHC_ACC_ADDRESS,0x23,0x20); // REG4_A, scale +/- 8G
  i2c_writeReg(LSM303DLHC_ACC_ADDRESS,0x21,0x00); // REG2_A, filter off

  acc_1G = 256;
}

  void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  i2c_getSixRawADC(LSM303DLHC_ACC_ADDRESS,0xA8);

  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>4 ,
                   ((rawADC[3]<<8) | rawADC[2])>>4 ,
                   ((rawADC[5]<<8) | rawADC[4])>>4 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Gyroscope L3GD20 
// ************************************************************************************************************
#if defined(L3GD20)
#define L3GD20_ADDRESS (0xD6 >> 1)
void Gyro_init() {
  delay(100);
  i2c_writeReg(L3GD20_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   380Hz ODR, 20hz filter, run!
  i2c_writeReg(L3GD20_ADDRESS ,0x23 ,0x0 );  // 250dps scale
  delay(5);
  i2c_writeReg(L3GD20_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(L3GD20_ADDRESS,0x80|0x28);

  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/20  ,
                    ((rawADC[3]<<8) | rawADC[2])/20  ,
                    ((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}
#endif

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  magADC[ROLL]  = magADC[ROLL]  * magGain[ROLL];
  magADC[PITCH] = magADC[PITCH] * magGain[PITCH];
  magADC[YAW]   = magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = magADC[axis];
      magZeroTempMax[axis] = magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    magADC[ROLL]  -= global_conf.magZero[ROLL];
    magADC[PITCH] -= global_conf.magZero[PITCH];
    magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      writeGlobalSet(1);
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((magADC[PITCH] - magADC[ROLL] )*7)/10;
      magADC[ROLL] = ((magADC[ROLL]  + magADC[PITCH])*7)/10;
      magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((magADC[PITCH] + magADC[ROLL] )*7)/10;
      magADC[ROLL] = ((magADC[ROLL]  - magADC[PITCH])*7)/10;
      magADC[PITCH] = temp;
    #endif
  }
  return 1;
}
#endif

// ************************************************************************************************************
// I2C Compass LSM303DLHC_MAG
// ************************************************************************************************************
// I2C adress: 0x1E (7bit)
// ************************************************************************************************************
#if defined(LSM303DLHC_MAG)
  #define MAG_ADDRESS 0x1E
  #define MAG_DATA_REGISTER 0x03
  #define MAG_CTRL_REG1 0x2
  
  void Mag_init() {
    delay(100);
    i2c_writeReg(MAG_ADDRESS,MAG_CTRL_REG1,0x00);  // continuous conversion mode
    magInit = 1;
  }
  
  void Device_Mag_getADC() {
    i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                     ((rawADC[2]<<8) | rawADC[3]) ,     
                     ((rawADC[4]<<8) | rawADC[5]) );
  }
#endif

// ************************************************************************************************************
// I2C Sonar SRF08
// ************************************************************************************************************
// first contribution from guru_florida (02-25-2012)
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(SRF08)

// the default address for any new sensor found on the bus
// the code will move new sonars to the next available sonar address in range of F0-FE so that another
// sonar sensor can be added again.
// Thus, add only 1 sonar sensor at a time, poweroff, then wire the next, power on, wait for flashing light and repeat
#if !defined(SRF08_DEFAULT_ADDRESS) 
  #define SRF08_DEFAULT_ADDRESS 0x70
#endif

#if !defined(SRF08_RANGE_WAIT) 
  #define SRF08_RANGE_WAIT     100000      // delay between Ping and Range Read commands
#endif

#if !defined(SRF08_RANGE_SLEEP) 
  #define SRF08_RANGE_SLEEP    1      // sleep this long before starting another Ping
#endif

#if !defined(SRF08_SENSOR_FIRST) 
  #define SRF08_SENSOR_FIRST    0x70      // inekhay: 7-bit now!
                                          // the first sensor i2c address (after it has been moved)
#endif

// #define SONAR_MULTICAST_PING

// registers of the device
#define SRF08_REV_COMMAND    0
#define SRF08_LIGHT_GAIN     1
#define SRF08_ECHO_RANGE     2

// read uncompensated temperature value: send command first
void Sonar_init() {
  memset(&srf08_ctx, 0, sizeof(srf08_ctx));
  srf08_ctx.deadline = 4000000;
}

// this function works like readReg accept a failed read is a normal expectation
// use for testing the existence of sensors on the i2c bus
// a 0xffff code is returned if the read failed
uint16_t i2c_try_readReg(uint8_t add, uint8_t reg) {
  uint16_t count = 255;
  i2c_rep_start(add<<1);  // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add<<1)|1);  // I2C read direction
  
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      return 0xffff;  // return failure to read
    }
  }
  
  uint8_t r = TWDR;
  i2c_stop();
  return r;  
}

// read a 16bit unsigned int from the i2c bus
uint16_t i2c_readReg16(int8_t addr, int8_t reg) {
  uint8_t b[2];
  i2c_read_reg_to_buf(addr, reg, &b, sizeof(b));
  return (b[0]<<8) | b[1];
}

/*
void i2c_srf08_change_addr(int8_t current, int8_t moveto) {
  // to change a srf08 address, we must write the following sequence to the command register
  // this sequence must occur as 4 seperate i2c transactions!!
  //   A0 AA A5 [addr]
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xA0);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xAA);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, 0xA5);  delay(30);
  i2c_writeReg(current, SRF08_REV_COMMAND, moveto);  delay(30); // now change i2c address
  blinkLED(5,1,2);
  #if defined(BUZZER)
   alarmArray[7] = 2;
  #endif
}
*/

// discover previously known sensors and any new sensor (move new sensors to assigned area)
void i2c_srf08_discover() {
  uint8_t addr;
  uint16_t x;

  // determine how many sensors are plugged in
  srf08_ctx.sensors=3;
  // inekhay: commented out 'cause not needed 
  // addr = SRF08_SENSOR_FIRST;
  // for(int i=0; i<SRF08_MAX_SENSORS && x!=0xff; i++) {
    // read the revision as a way to check if sensor exists at this location
    // x = i2c_try_readReg(addr, SRF08_REV_COMMAND);
    // if(x!=0xffff) {
      // detected a sensor at this address
      // srf08_ctx.sensors++;
      // addr += 2;
    // }
  // }
  
  // do not add sensors if we are already maxed
  // if(srf08_ctx.sensors < SRF08_MAX_SENSORS) {
    // now determine if any sensor is on the 'new sensor' address (srf08 default address)
    // we try to read the revision number
    // x = i2c_try_readReg(SRF08_DEFAULT_ADDRESS, SRF08_REV_COMMAND);
    // if(x!=0xffff) {
      // new sensor detected at SRF08 default address

      // inekhay: Commented out for MB1242!!
      // i2c_srf08_change_addr(SRF08_DEFAULT_ADDRESS, addr);  // move sensor to the next address
      // srf08_ctx.sensors++;
    // }
  // }
}

void Sonar_update() {
  if (currentTime < srf08_ctx.deadline || (srf08_ctx.state==0 && f.ARMED)) return; 
  srf08_ctx.deadline = currentTime;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, SRF08 is ok with this speed
  switch (srf08_ctx.state) {
    case 0: 
      i2c_srf08_discover();
      if(srf08_ctx.sensors > 0)
        srf08_ctx.state++; 
      else
        srf08_ctx.deadline += 5000000; // wait 5 secs before trying search again
      break;
    case 1: 
      srf08_ctx.current=0;
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_SLEEP;
      break;
/*
#if defined(SONAR_MULTICAST_PING)
    case 2:
      // send a ping via the general broadcast address
      i2c_writeReg(0, SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_WAIT;
      break;
    case 3: 
      srf08_ctx.range[srf08_ctx.current] = i2c_readReg16(SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_ECHO_RANGE);
      srf08_ctx.current++;
      if(srf08_ctx.current >= srf08_ctx.sensors)
        srf08_ctx.state=1;
      break;
#else
*/
    case 2:
      // send a ping to the current sensor

      // inekhay: we'll have 3 MB1242s with changed addresses
      i2c_rep_start((SRF08_SENSOR_FIRST + srf08_ctx.current) << 1); // I2C write direction, 
      i2c_write(0x51);        // start ranging, result in centimeters
      i2c_stop();
      
      srf08_ctx.state++;
      srf08_ctx.deadline += SRF08_RANGE_WAIT;
      break;
    case 3: 
      // inekhay:
      // srf08_ctx.range[srf08_ctx.current] = i2c_readReg16(SRF08_SENSOR_FIRST+(srf08_ctx.current<<1), SRF08_ECHO_RANGE);
      
      uint8_t b[2];
      if(2 == i2c_read_to_buf((SRF08_SENSOR_FIRST + srf08_ctx.current), &b, sizeof(b))) {
        int32_t range = (b[0]<<8) | b[1];
        if( srf08_ctx.current == 0 || (range > 23 && range < 765) ) {
          srf08_ctx.range[srf08_ctx.current] = range;
          srf08_ctx.readAt[srf08_ctx.current] = currentTime;
        }
      }

      srf08_ctx.current++;
      if(srf08_ctx.current >= srf08_ctx.sensors)
        srf08_ctx.state=1;
      else
        srf08_ctx.state=2; 
      break;
// #endif
  } 
  sonarAlt = srf08_ctx.range[0]; //tmp

  // debug[0] = srf08_ctx.range[0];
  // debug[1] = srf08_ctx.range[1];
  // debug[2] = srf08_ctx.range[2];  
}
#else
inline void Sonar_init() {}
inline void Sonar_update() {}
#endif


#if defined(BARO)
void  Baro_init() {
}

void Baro_Common() {
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {
  return 0;
}
#endif

void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  if (BARO) Baro_init();
  if (MAG) Mag_init();
  if (ACC) {ACC_init();acc_25deg = acc_1G * 0.423;}
  if (SONAR) Sonar_init();
  f.I2C_INIT_DONE = 1;
}
