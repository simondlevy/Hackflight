/*
   Adapted from https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// Standard Arduino libraries
#include <Wire.h>
#include <SPI.h>

// Hackflight library
#include <hackflight.hpp>
#include <utils.hpp>
#include <timer.hpp>

// PID control ---------------------------------------------------------------
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>
static constexpr float THROTTLE_DOWN = 0.06;
static hf::YawRatePid _yawRatePid;
static hf::PitchRollAnglePid _pitchRollAnglePid;
static hf::PitchRollRatePid _pitchRollRatePid;

// Sensor fusion -------------------------------------------------------------
#include <estimators/madgwick.hpp>
static hf::MadgwickFilter _madgwick;

// Blinkenlights -------------------------------------------------------------
#include <TinyPICO.h>
static TinyPICO _tinypico;
static const uint32_t LED_FAILSAFE_COLOR = 0xFF0000;
static const uint32_t LED_HEARTBEAT_COLOR = 0x00FF00;
static const uint32_t LED_ARMED_COLOR = 0xFF0000;
static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;

// Motors --------------------------------------------------------------------
#include <oneshot125.hpp>
const std::vector<uint8_t> MOTOR_PINS = { 23, 26, 7, 15 };
static uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;

// Receiver ------------------------------------------------------------------
#include <sbus.h>
static bfs::SbusRx _rx = bfs::SbusRx(&Serial1, 4, 14, true);

// IMU -----------------------------------------------------------------------
#include <MPU6050.h>
static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131.0;
static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384.0;
static MPU6050 _mpu6050;

// Mixing
#include <mixers/bfquadx.hpp>
static hf::BfQuadXMixer _mixer;

// FAFO -----------------------------------------------------------
static const uint32_t LOOP_FREQ_HZ = 2000;

// Helpers --------------------------------------------------------------------

static void reportForever(const char * message)
{
    while (true) {
        printf("%s\n", message);
        delay(500);
    }
}

static void runLoopDelay(const uint32_t usec_curr)
{
    //DESCRIPTION: Regulate main loop rate to specified frequency
    //in Hz
    /*
     * It's good to operate at a constant loop rate for filters to
     * remain stable and whatnot. Interrupt routines running in the
     * background cause the loop rate to fluctuate. This function
     * basically just waits at the end of every loop iteration
     * until the correct time has passed since the start of the
     * current loop for the desired loop rate in Hz. 2kHz is a good
     * rate to be at because the loop nominally will run between
     * 2.8kHz - 4.2kHz. This lets us have a little room to add
     * extra computations and remain above 2kHz, without needing to
     * retune all of our filtering parameters.
     */
    float invFreq = 1.0/LOOP_FREQ_HZ*1000000.0;

    unsigned long checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - usec_curr)) {
        checker = micros();
    }
}

static void blinkLed(const uint32_t usec_curr, const bool gotFailsafe)
{
    const auto freq_hz =
        gotFailsafe ?
        FAILSAFE_BLINK_RATE_HZ :
        HEARTBEAT_BLINK_RATE_HZ;

    const auto color = gotFailsafe ? LED_FAILSAFE_COLOR : LED_HEARTBEAT_COLOR;

    static uint32_t _usec_prev;

    static uint32_t _delay_usec;

    if (usec_curr - _usec_prev > _delay_usec) {

        static bool _alternate;

        _usec_prev = usec_curr;

        _tinypico.DotStar_SetPixelColor(_alternate ? color : 0x000000);

        if (_alternate) {
            _alternate = false;
            _delay_usec = 100'100;
        }

        else {
            _alternate = true;
            _delay_usec = freq_hz * 1e6;
        }
    }
}


// Setup ----------------------------------------------------------------------

void setup() 
{
    Serial.begin(115200);

    Wire.begin();

    // Note this is 2.5 times the spec sheet 400 kHz max...
    Wire.setClock(1000000); 

    _mpu6050.initialize();

    if (!_mpu6050.testConnection()) {
        reportForever("MPU6050 initialization unsuccessful\n");
    }

    // From the reset state all registers should be 0x00, so we
    // should be at max sample rate with digital low pass filter(s)
    // off.  All we need to do is set the desired fullscale ranges
    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

    _rx.Begin();

    // Initialize the Madgwick filter
    _madgwick.initialize();
}

static void readReceiver(uint16_t channels[6], bool & gotFailsafe) 
{
    if (_rx.Read()) {

        const auto data = _rx.data();

        if (data.failsafe) {

            gotFailsafe = true;
        }

        else {
            channels[0] = data.ch[0];
            channels[1] = data.ch[1];
            channels[2] = data.ch[2];
            channels[3] = data.ch[3];
            channels[4] = data.ch[4];
            channels[5] = data.ch[5];
        }
    }
}


// Loop ----------------------------------------------------------------------

void loop() 
{
    static uint16_t _channels[6];
    static bool _isArmed;
    static bool _gotFailsafe;

    const auto usec_curr = micros();      

    // Read receiver
    readReceiver(_channels, _gotFailsafe);

    // Disarm immiedately on failsafe
    if (_gotFailsafe) {
        _isArmed = false;
    }

    printf("c1=%04d c2=%04d c3=%04d c4=%04d c5=%04d c6=%04d\n",
            _channels(0), _channels(1), _channels(2), _channels(3), _channels(4), _channels(5));


    // Arm vehicle if safe
    //if (!_gotFailsafe && (_channels[4] > 1500) && (_channels[0] < 1050)) {
    //    _isArmed = true;
    //}


    // Run PID controllers
    //const auto resetPids = demands.thrust < THROTTLE_DOWN;
    //_pitchRollAnglePid.run(dt, resetPids, state, demands);
    //_pitchRollRatePid.run(dt, resetPids, state, demands);
    //_yawRatePid.run(dt, resetPids, state, demands);
    //float motors[4] = {};

    // Mix motors
    //_mixer.run(demands, motors);

    // LED should be on when armed
    if (_isArmed) {
        _tinypico.DotStar_SetPixelColor(LED_ARMED_COLOR);
    }

    // Otherwise, blink LED as heartbeat or failsafe rate
    else {
        blinkLed(usec_curr, _gotFailsafe);
    }

    runLoopDelay(usec_curr);
}
