/*
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

// DotStar LED support
#include <TinyPICO.h>

// OneShot125 support
#include <oneshot125.hpp>

// IMU support
#include <MPU6050.h>

// Hackflight library
#include <hackflight.hpp>
#include <utils.hpp>
#include <timer.hpp>
#include <estimators/madgwick.hpp>
#include <mixers/bfquadx.hpp>

#include <sbus.h>

namespace hf {

    class Board {

        private:

            // Sensor fusion -------------------------------------------------------------
            hf::MadgwickFilter _madgwick;

            // Blinkenlights -------------------------------------------------------------
            static const uint32_t LED_FAILSAFE_COLOR = 0xFF0000;
            static const uint32_t LED_HEARTBEAT_COLOR = 0x00FF00;
            static const uint32_t LED_ARMED_COLOR = 0xFF0000;
            static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
            static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;
            TinyPICO _tinypico;

            // Motors --------------------------------------------------------------------
            const std::vector<uint8_t> MOTOR_PINS = { 23, 26, 7, 15 };
            uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;

            // Receiver ------------------------------------------------------------------
            static const uint16_t CHAN5_ARM_MIN = 1000;
            static const uint16_t CHAN1_ARM_MAX = 180;
            bfs::SbusRx _rx = bfs::SbusRx(&Serial1, 4, 14, true);

            // IMU -----------------------------------------------------------------------
            static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
            static constexpr float GYRO_SCALE_FACTOR = 131.0;
            static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
            static constexpr float ACCEL_SCALE_FACTOR = 16384.0;
            static constexpr float ACC_ERROR_X = 0.0;
            static constexpr float ACC_ERROR_Y = 0.0;
            static constexpr float ACC_ERROR_Z = 0.0;
            static constexpr float GYRO_ERROR_X = 0.0;
            static constexpr float GYRO_ERROR_Y= 0.0;
            static constexpr float GYRO_ERROR_Z = 0.0;
            MPU6050 _mpu6050;

            // Mixing
            hf::BfQuadXMixer _mixer;

            // Debugging
            static constexpr float DEBUG_RATE_HZ = 100;

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

            void blinkLed(const uint32_t usec_curr, const bool gotFailsafe)
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

        public:

            void init() 
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

            void readReceiver(uint16_t channels[6], bool & gotFailsafe) 
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

            void step() 
            {
                static uint16_t _channels[6];
                static bool _isArmed;
                static bool _gotFailsafe;
                static uint32_t _usec_prev;
                static hf::Timer _debugTimer;

                const auto usec_curr = micros();      

                const float dt = (usec_curr - _usec_prev)/1000000.0;

                _usec_prev = usec_curr;

                // Read receiver
                readReceiver(_channels, _gotFailsafe);

                // Disarm immiedately on failsafe
                if (_gotFailsafe) {
                    _isArmed = false;
                }


                // Arm vehicle if safe
                if (!_gotFailsafe &&
                        (_channels[4] > CHAN5_ARM_MIN) && (_channels[0] < CHAN1_ARM_MAX)) {
                    _isArmed = true;
                }

                // Disarm when requested
                if (_channels[4] < CHAN5_ARM_MIN) {
                    _isArmed = false;
                }

                // Read IMU
                int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
                _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                // Accelerometer Gs
                const hf::axis3_t accel = {
                    -ax / ACCEL_SCALE_FACTOR - ACC_ERROR_X,
                    ay / ACCEL_SCALE_FACTOR - ACC_ERROR_Y,
                    az / ACCEL_SCALE_FACTOR - ACC_ERROR_Z
                };

                // Gyro deg /sec
                const hf::axis3_t gyro = {
                    gx / GYRO_SCALE_FACTOR - GYRO_ERROR_X, 
                    -gy / GYRO_SCALE_FACTOR - GYRO_ERROR_Y,
                    -gz / GYRO_SCALE_FACTOR - GYRO_ERROR_Z
                };

                // Run Madgwick filter to get get Euler angles from IMU values
                // (note negations)
                hf::axis4_t quat = {};
                _madgwick.getQuaternion(dt, gyro, accel, quat);

                // Compute Euler angles from quaternion
                hf::axis3_t angles = {};
                hf::Utils::quat2euler(quat, angles);

                if (_debugTimer.isReady(usec_curr, DEBUG_RATE_HZ)) {
                    /*
                       printf("c1=%04d c2=%04d c3=%04d c4=%04d c5=%04d c6=%04d\n",
                       _channels[0], _channels[1], _channels[2], _channels[3], _channels[4], _channels[5]); */
                    printf("phi=%+3.3f  theta=%+3.3f  psi=%+3.3f\n", angles.x, angles.y, angles.z);
                    //printf("ax=%+3.3f  ay=%+3.3f  az=%+3.3f\n", accel.x, accel.y, accel.z);

                }

                //state.phi = angles.x;
                //state.theta = angles.y;
                //state.psi = angles.z;

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

    }; // class Board

} // namespace hf
