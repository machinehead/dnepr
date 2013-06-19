/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
    #if (NUMBER_MOTOR > 0)
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]>>3; //  pin 9
      #else
        OCR1A = ((motor[0]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]>>3; //  pin 10
      #else
        OCR1B = ((motor[1]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE
        OCR2A = motor[2]>>3; //  pin 11
      #else
        OCR2A = ((motor[2]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE
        OCR2B = motor[3]>>3; //  pin 3
      #else
        OCR2B = ((motor[3]>>2) - 250);
      #endif
    #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    #endif
    #if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
      initializeSoftPWM();
      #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
        pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
        pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
      #endif
    #endif

  /********  special version of MultiWii to calibrate all attached ESCs ************/
  #if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    delay(3000);
    writeAllMotors(ESC_CALIB_LOW);
    delay(500);
    while (1) {
      delay(5000);
      blinkLED(2,20, 2);
    #if defined(BUZZER)
      alarmArray[7] = 2;
    #endif
    }
    exit; // statement never reached
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
}

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  
      motor[0] = PIDMIX(-1,+1,-1); //REAR_R
      motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
      motor[2] = PIDMIX(+1,+1,+1); //REAR_L
      motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  
  /****************                Filter the Motors values                ******************/
  #ifdef GOVERNOR_P
    if (rcOptions[BOXGOV] ) {
      static int8_t g[37] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
      uint8_t v = constrain( VBATNOMINAL - constrain(vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, 36);
      for (i = 0; i < NUMBER_MOTOR; i++) {
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) * (int32_t)conf.governorR )/ 5000;
      }
    }
  #endif
  /****************                normalize the Motors values                ******************/
  #ifdef LEAVE_HEADROOM_FOR_MOTORS
    // limit this leaving room for corrections to the first #n of all motors
    maxMotor=motor[0];
    for(i=1; i < LEAVE_HEADROOM_FOR_MOTORS; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    if (maxMotor > MAXTHROTTLE) { // this is a way to still have good gyro corrections if at least one motor reaches its max.
      for(i=0; i < LEAVE_HEADROOM_FOR_MOTORS; i++)
        motor[i] -= maxMotor - MAXTHROTTLE;
    }
    for (i = 0; i < NUMBER_MOTOR; i++) {
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      #if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
        if (rcData[THROTTLE] < MINCHECK)
      #else
        if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
      #endif
        #ifndef MOTOR_STOP
          motor[i] = conf.minthrottle;
        #else
          motor[i] = MINCOMMAND;
        #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }
  #else // LEAVE_HEADROOM_FOR_MOTORS
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      #if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
        if (rcData[THROTTLE] < MINCHECK)
      #else
        if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
      #endif
        #ifndef MOTOR_STOP
          motor[i] = conf.minthrottle;
        #else
          motor[i] = MINCOMMAND;
        #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }
  #endif // LEAVE_HEADROOM_FOR_MOTORS
  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
    uint16_t amp, ampsum;
    /* true cubic function;
     * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
     * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
     * */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (vbat > conf.no_vbat) { // by all means - must avoid division by zero
      ampsum = 0;
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
           #if (LOG_VALUES >= 3)
           pMeter[i]+= amp; // sum up over time the mapped ESC input 
        #endif
        #if defined(POWERMETER_SOFT)
           ampsum += amp; // total sum over all motors
        #endif
      }
      #if defined(POWERMETER_SOFT)
        pMeter[PMOTOR_SUM]+= ampsum; // total sum over all motors
      #endif
    }
  #endif
}
