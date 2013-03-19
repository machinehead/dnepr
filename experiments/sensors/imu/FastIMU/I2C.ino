/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

void I2C_Init()
{
  Wire.begin();
}

void Gyro_Init()
{
  gyro.init();
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
}

void Accel_Init()
{
  compass.init();
  if (compass.getDeviceType() == LSM303DLHC_DEVICE)
  {
    compass.writeAccReg(LSM303_CTRL_REG1_A, 0x47); // normal power mode, all axes enabled, 50 Hz
    // compass.writeAccReg(LSM303_CTRL_REG2_A, 0x88); // HPF enable at lowest freq, filtered data selected
    compass.writeAccReg(LSM303_CTRL_REG4_A, 0x08); // 2g scale, high resolution output mode
  }
}

void Compass_Init()
{
  compass.writeMagReg(LSM303_MR_REG_M, 0x00); // continuous conversion mode
  // 15 Hz default
}

void Read_Gyro()
{
  gyro.read();
  
  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
  compass.readAcc();
  
  AN[3] = compass.a.x;
  AN[4] = compass.a.y;
  AN[5] = compass.a.z;
}

void Read_Compass()
{
  compass.readMag();
  
  AN[6] = compass.m.x;
  AN[7] = compass.m.y;
  AN[8] = compass.m.z;
}

