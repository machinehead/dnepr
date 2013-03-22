#include <Wire.h>

#define READ_FREQ 50

#define STATUS_LED 13 

long timer=0;   //general purpuse timer
long timer_old;
// long timer24=0; //Second timer used to print values 

float G_Dt=0.02;    // Integration time

int AN[9]; //array that stores the gyro and accelerometer data

unsigned int counter=0;

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
  
  delay(500);
  digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
}

void loop() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    Read_Gyro();      // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
    }
 
    printdata();
  }
   
}
