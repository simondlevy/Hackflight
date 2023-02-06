/*
   MPU6x00 IMU on Teensy MCU

   Copyright (c) 2023 Simon D. Levy

   MIT License
 */

#include <hackflight.h>
#include <board/teensy40.h>

#include <mpu6x00.h>

static const uint8_t CS_PIN  = 10;
static const uint8_t INT_PIN = 9;

static Mpu6x00 mpu = Mpu6x00(CS_PIN);

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

    if (!mpu.begin()) {
        errorForever();
    }

    pinMode(INT_PIN, INPUT);
    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void)
{
    if (gotInterrupt) {

        mpu.readSensor();

        Serial.printf(
                "gx=%+06d gy=%+06d gz=%+06d\n",
                mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawAccelZ());

        gotInterrupt = false;
    }
}
