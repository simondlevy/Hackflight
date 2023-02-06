/*
   MPU6x00 IMU on Teensy MCU

   Copyright (c) 2023 Simon D. Levy

   MIT License
 */

#include <hackflight.h>
#include <board/teensy40.h>

#include <mpu6x00.h>

static const uint8_t IMU_CS_PIN  = 10;
static const uint8_t IMU_INT_PIN = 9;

static Mpu6x00 mpu = Mpu6x00(IMU_CS_PIN);

static bool gotInterrupt;

static void handleImuInterrupt(void)
{
    gotInterrupt = true;
}

void setup(void)
{
    Serial.begin(115200);

    SPI.begin();

    mpu.begin();

    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);
}

void loop(void)
{
    if (gotInterrupt) {

        mpu.readSensor();

        Serial.printf("gx=%+06d gy=%+06d gz=%+06d\n",
                mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ());

        gotInterrupt = false;
    }
}
