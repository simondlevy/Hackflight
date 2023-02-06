/*
   MPU6x00 IMU on Teensy MCU

   Copyright (c) 2023 Simon D. Levy

   MIT License
 */

#include "mpu6x00.h"

static const uint8_t CS_PIN  = 10;
static const uint8_t INT_PIN = 9;

static Mpu6x00 imu = Mpu6x00(CS_PIN);

static bool gotInterrupt;

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

    SPI.begin();

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

        Serial.printf(
                "gx=%+06d gy=%+06d gz=%+06d | ax=%+06d ay=%+06d az=%+06d\n",
                imu.getRawGyroX(),
                imu.getRawGyroY(),
                imu.getRawGyroZ(),
                imu.getRawAccelX(),
                imu.getRawAccelY(),
                imu.getRawAccelZ());

        gotInterrupt = false;
    }
}
