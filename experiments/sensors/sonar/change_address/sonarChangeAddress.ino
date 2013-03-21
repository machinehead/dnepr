#include <Wire.h>

const byte SensorAddress1 = 0x70;
const byte SensorAddress2 = 0x71;

const byte  RangeCommand = 0x51;

//These are the two commands that need to be sent in sequence to change the sensor address
const byte  ChangeAddressCommand1 = 0xAA;
const byte  ChangeAddressCommand2 = 0xA5;

// set pin numbers:
const int buttonPin = 2; // the number of the pushbutton pin
const int ledPin =  13; // the number of the LED pin

bool isMeasuring = false;
bool isChanged = false;

void setup() 
{
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);    
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
   //Open serial connection at 9600 baud
  Serial.begin(9600);
  //Initiate Wire library for I2C communications with the I2CXL-MaxSonar-EZ
  Wire.begin(); 
}

void loop()
{
  int buttonState = digitalRead(buttonPin);
    
  
  if( buttonState == HIGH ) {
    if( !isChanged ) {
      changeAddress( SensorAddress1, SensorAddress2 );
      Serial.print( "Change sensor address done.\r\n" );
      isChanged = true;
    }
    
    digitalWrite(ledPin, HIGH); // turn LED on
    isMeasuring = true;
    takeRangeReading(SensorAddress2);
    delay(100); //Wait for sensor to finish
    word range = requestRange(SensorAddress2); //Get the range from the sensor
    
    Serial.print(range);
    Serial.print("; ");
  } else {
    if( isMeasuring ) {
      Serial.print("\r\n");
      isMeasuring = false;
    }
    // turn LED off:
    digitalWrite(ledPin, LOW);
    delay(100);  
  }
}

//Commands the sensor to take a range reading
void takeRangeReading( byte sensorAddress )
{  
  Wire.beginTransmission(sensorAddress); //Start addressing
  Wire.write(RangeCommand); //send range command
  Wire.endTransmission(); //Stop and do something else now
}

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
word requestRange( byte sensorAddress )
{
  Wire.requestFrom(sensorAddress, byte(2));
  if(Wire.available() >= 2){ //Sensor responded with the two bytes
    byte HighByte = Wire.read(); //Read the high byte back
    byte LowByte = Wire.read(); //Read the low byte back
    word range = word(HighByte, LowByte); //Make a 16-bit word out of the two bytes for the range
    return range;
  } else {
    return word(0); //Else nothing was received, return 0
  }
}

void changeAddress( byte oldAddress, byte newAddress ){
  Wire.beginTransmission(oldAddress);
  Wire.write(ChangeAddressCommand1); //Send first change address command
  Wire.write(ChangeAddressCommand2); //Send second change address command
  byte eightBitAddress = newAddress << 1;
  Wire.write(eightBitAddress); //Send the new address to change to
  Wire.endTransmission();
}
