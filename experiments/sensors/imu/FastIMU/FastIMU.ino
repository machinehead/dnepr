#include <Wire.h>

#define READ_FREQ 100
#define MAIN_LOOP_DELAY (1000 / READ_FREQ)

#define STATUS_LED 13 

long timer=0;   //general purpuse timer
long timer_old;
// long timer24=0; //Second timer used to print values 

float G_Dt = (1 / float(READ_FREQ));    // Integration time

int AN[9]; //array that stores the gyro and accelerometer data

// current sonar range value in cm
word sonarRange=0;
// time since last sonar ping, ms
static long sonarPingTimer=0;
// 10 Hz sonar reading
#define SONAR_PING_DELAY 100

// Set to 1 on each sonar reading and reset to 0 on each output.
byte sonarNew = 0;
// Set to 1 on each sonar ping call and reset to 0 after data has been read.
byte sonarPingRead = 1;

void setup()
{ 
  Serial.begin(115200);
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  I2C_Init();

  Serial.println("FastIMU");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);
 
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  Sonar_Init();
  
  delay(500);
  digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
}

void loop() //Main Loop
{
  if((millis()-timer) >= MAIN_LOOP_DELAY)
  {
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    Read_Gyro();      // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    if(sonarPingRead) {
      Sonar_Read();     // Read sonar
      if(sonarNew)
        sonarPingRead = 0;
    }
      
    
    if((millis() - sonarPingTimer) >= SONAR_PING_DELAY)
    {
      Sonar_Ping();
      sonarPingTimer = millis();
      sonarPingRead = 1;
      // Also read compass data at 10Hz...
      Read_Compass();    // Read I2C magnetometer
    }
 
    printdata();
  }
   
}
