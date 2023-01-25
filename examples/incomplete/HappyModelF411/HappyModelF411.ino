/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <imu/softquat/invensense/icm20689.h>

static const uint8_t IMU_MOSI_PIN = PA7;
static const uint8_t IMU_MISO_PIN = PA6;
static const uint8_t IMU_SCLK_PIN = PA5;

static const uint8_t LED_PIN     = PC13;
static const uint8_t IMU_CS_PIN  = PA4;
static const uint8_t IMU_INT_PIN = PA1;

static SPIClass spi = SPIClass(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_SCLK_PIN);

static ICM20689 imu(spi, IMU_CS_PIN);

static volatile bool gotInterrupt;

static void getIMU() 
{
    gotInterrupt = true;
}

static void blinkLed(void)
{
    static uint32_t prev;
    static bool led;

    uint32_t msec = millis();

    if (msec-prev > 500) {
        led = !led;
        digitalWrite(LED_PIN, led);
        prev = msec;
    }
}

void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  imu.begin();

  pinMode(IMU_INT_PIN,INPUT);
  attachInterrupt(IMU_INT_PIN,getIMU,RISING);

}

void loop() 
{
    blinkLed();

    if (gotInterrupt) {

        gotInterrupt = false;

        imu.readSensor();

        Serial.print(imu.accelCounts[0]);
        Serial.print("\t");
        Serial.print(imu.accelCounts[1]);
        Serial.print("\t");
        Serial.print(imu.accelCounts[2]);
        Serial.print("\t");
        Serial.print(imu.gyroCounts[0]);
        Serial.print("\t");
        Serial.print(imu.gyroCounts[1]);
        Serial.print("\t");
        Serial.print(imu.gyroCounts[2]);
        Serial.print("\n");
    }
}
