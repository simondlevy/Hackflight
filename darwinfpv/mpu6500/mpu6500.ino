/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <mpu6x00.hpp>

static const uint8_t SCLK_PIN = PA5;
static const uint8_t MISO_PIN = PA6;
static const uint8_t MOSI_PIN = PA7;

static const uint8_t CS_PIN  = PA4;
static const uint8_t INT_PIN = PA1;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static Mpu6500 imu = Mpu6500(spi, CS_PIN);

static volatile bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static void errorForever(void)
{
    while (true) {
        Serial.println("Error initializing IMU");
        delay(500);
    }
}

void setup(void)
{
    Serial.begin(115200);

    spi.begin();

    if (!imu.begin()) {
        errorForever();
    }

    pinMode(INT_PIN, INPUT);
    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void)
{
    if (gotInterrupt) {

        imu.readSensor();

        Serial.print("ax=");
        Serial.print(imu.getRawAccelX());
        Serial.print("  ay=");
        Serial.print(imu.getRawAccelY());
        Serial.print("  az=");
        Serial.print(imu.getRawAccelZ());
        Serial.print(" gx=");
        Serial.print(imu.getRawGyroX());
        Serial.print("  gy=");
        Serial.print(imu.getRawGyroY());
        Serial.print("  gz=");
        Serial.print(imu.getRawGyroZ());
        Serial.println(); 

        gotInterrupt = false;
    }
}
