/*
   MPU6050_raw : example of reading raw IMU data from MPU6050

   This file is part of BreezySTM32.

   MPU6050 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <MPU6050.h>

MPU6050 * imu;

void setup()
{
    Serial.begin(115200);

    Wire.begin(2);

    imu = new MPU6050();
 
    if (!imu->begin(AFS_2G, GFS_250DPS)) {
        Serial.printf("MPU6050 is online...\n");
    }
    else {
        Serial.printf("Failed to init MPU6050\n");
        while (true) 
            ;
    }
}

void loop()
{  
    int16_t ax, ay, az, gx, gy, gz;

    if (imu->getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz)) {
        Serial.printf("%d %d %d %d %d %d\n", ax, ay, az, gx, gy, gz);
    }
}
