/*
   MPU6x00 IMU on Teensy MCU

   Copyright (c) 2023 Simon D. Levy

   MIT License
 */

#include <hackflight.h>
#include <board/teensy40.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <core/pids/angle.h>
#include <imu/softquat.h>
#include <esc/mock.h>

#include <vector>

#include <SPI.h>
#include <mpu6x00.h>


static const uint8_t IMU_CS_PIN  = 10;
static const uint8_t IMU_INT_PIN = 9;
static const uint8_t LED_PIN     = 0; 

static Mpu6x00 mpu = Mpu6x00(IMU_CS_PIN);

static SoftQuatImu imu(Imu::rotate270);

static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static std::vector<PidController *> pids = {&anglePid};

static Mixer mixer = QuadXbfMixer::make();

static MockEsc esc;

static Teensy40 board(imu, pids, mixer, esc, LED_PIN);

static bool gotImuInterrupt;

static void handleImuInterrupt(void)
{
    gotImuInterrupt = true;
    // board.handleImuInterrupt();
}

void setup(void)
{
    Board::setInterrupt(IMU_INT_PIN, handleImuInterrupt, RISING);

    Serial.begin(115200);

    SPI.begin();

    mpu.begin();
}

void loop(void)
{
    if (gotImuInterrupt) {

        gotImuInterrupt = false;

        mpu.readSensor();

        Serial.printf("gx=%+06d gy=%+06d gz=%+06d\n",
                mpu.getRawGyroX(), mpu.getRawGyroY(), mpu.getRawGyroZ());
    }
}
