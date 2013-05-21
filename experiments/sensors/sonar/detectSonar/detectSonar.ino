#include <Wire.h>

/* Code for Arduino Uno R3
Assumes the sensor is using the default address
Sensor Connections:
Pin 7 to GND
Pin 6 to 5V
Pin 5 to SCL
Pin 4 to SDA
Requires pull-ups for SCL and SDA connected to 5V to work reliably
*/

//The sensors ranging command has a value of 0x51
const byte  RangeCommand = 0x51;
//These are the two commands that need to be sent in sequence to change the sensor address
const byte  ChangeAddressCommand1 = 0xAA;
const byte  ChangeAddressCommand2 = 0xA5;

// set pin numbers:
const int ledPin =  13;      // the number of the LED pin

//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8-bit 0xE0
byte CurrentSensorAddress = 0x00; // this will change from 0x00 through 0x7F


void setup() 
{
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  //Open serial connection at 9600 baud
  Serial.begin(9600);
  //Initiate Wire library for I2C communications with the I2CXL-MaxSonar-EZ
  Wire.begin(); 
  Serial.println("Begin sonar scan..");
}

void loop()
{
  digitalWrite(ledPin, HIGH); // turn LED on
  takeRangeReading(); //Tell the sensor to perform a ranging cycle
  delay(100); //Wait for sensor to finish
  word range = requestRange(); //Get the range from the sensor
  if (range != 0) {
    Serial.print("0x");
    Serial.print(CurrentSensorAddress, HEX);
    Serial.print("; ");
    Serial.print(range);
    Serial.println("");
  }
  digitalWrite(ledPin, LOW); // turn LED off
  CurrentSensorAddress = (CurrentSensorAddress + 1) % 0x80;
}

//Commands the sensor to take a range reading
void takeRangeReading()
{  
  Wire.beginTransmission(CurrentSensorAddress); //Start addressing
  Wire.write(RangeCommand); //send range command
  Wire.endTransmission(); //Stop and do something else now
}

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
word requestRange()
{
  Wire.requestFrom(CurrentSensorAddress, byte(2));
  if(Wire.available() >= 2){ //Sensor responded with the two bytes
    byte HighByte = Wire.read(); //Read the high byte back
    byte LowByte = Wire.read(); //Read the low byte back
    word range = word(HighByte, LowByte); //Make a 16-bit word out of the two bytes for the range
    return range;
  } else {
    return word(0); //Else nothing was received, return 0
  }
}

