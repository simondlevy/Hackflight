/*

   Support for "turtle board" quadcopter using MPU6050 IMU and SBUS receiver

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

#pragma once

#include <Wire.h>

#include <sbus.h>
#include <MPU6050.h>
#include <oneshot125.hpp>

#include <hackflight.hpp>
#include <madgwick.hpp>
#include <utils.hpp>
#include <tasks/blink.hpp>

// Receiver -------------------------------------------------------------------

namespace hf {

    class TurtleBoardSbus {

        public:

            void init()
            {
                // Set up serial debugging
                Serial.begin(500000);
                delay(500);

                // Start receiver
                _rx.Begin();

                // Initialize LED
                pinMode(LED_PIN, OUTPUT); 
                digitalWrite(LED_PIN, HIGH);

                delay(5);

                // Indicate entering main loop with some quick blinks
                blinkOnStartup(); 

                // Initialize IMU communication
                initImu();

                delay(5);

                // Arm OneShot125 motors
                armMotor(_m4_usec);
                armMotor(_m2_usec);
                armMotor(_m1_usec);
                armMotor(_m3_usec);
                _motors.arm();
            }

            void readData(
                    float & dt,
                    float & thrustDemand, float & rollDemand, float & pitchDemand, float & yawDemand,
                    float & phi, float & theta, float & psi,
                    float & gyroX, float & gyroY, float & gyroZ
                    )
            {

                // Keep track of what time it is and how much time has elapsed since the last loop
                _usec_curr = micros();      
                static uint32_t _usec_prev;
                dt = (_usec_curr - _usec_prev)/1000000.0;
                _usec_prev = _usec_curr;      

                // Arm vehicle if safe
                if (!_gotFailsafe && (_chan_5 > 1500) && (_chan_1 < 1050)) {
                    _isArmed = true;
                }

                // LED should be on when armed
                if (_isArmed) {
                    digitalWrite(LED_PIN, HIGH);
                }

                // Otherwise, blink LED as heartbeat or failsafe rate
                else {
                    _blinkTask.run(LED_PIN, _usec_curr,
                            _gotFailsafe ? 
                            FAILSAFE_BLINK_RATE_HZ : 
                            HEARTBEAT_BLINK_RATE_HZ);
                }

                //Get vehicle state

                float AccX = 0, AccY = 0, AccZ = 0;

                readImu(AccX, AccY, AccZ, gyroX, gyroY, gyroZ); 

                // Get Euler angles from IMU (note negations)
                Madgwick6DOF(dt, gyroX, -gyroY, gyroZ, -AccX, AccY, AccZ, phi, theta, psi);
                psi = -psi;

                // Convert stick demands to appropriate intervals
                thrustDemand = sbusmap(_chan_1,  0.,  1.);
                rollDemand   = sbusmap(_chan_2, -1,  +1) * PITCH_ROLL_PRESCALE;
                pitchDemand  = sbusmap(_chan_3, -1,  +1) * PITCH_ROLL_PRESCALE;
                yawDemand    = sbusmap(_chan_4, -1,  +1) * YAW_PRESCALE;
            }

            static float sbusmap(
                    const uint16_t val, const float min, const float max)
            {
                return min + (val - 172.) / (1811 - 172) * (max - min);
            }

            void runMotors(
                    const float m1_command,
                    const float m2_command,
                    const float m3_command,
                    const float m4_command)
            {
                // Rescale motor values for OneShot125
                _m1_usec = scaleMotor(m1_command);
                _m2_usec = scaleMotor(m2_command);
                _m3_usec = scaleMotor(m3_command);
                _m4_usec = scaleMotor(m4_command);

                // Turn off motors under various conditions
                cutMotors(_chan_5, _isArmed); 

                // Run motors
                runMotors(); 

                // Get vehicle commands for next loop iteration
                readReceiver();

                runLoopDelay(_usec_curr);
            }

        private:

            // FAFO
            static const uint32_t LOOP_FREQ_HZ = 2000;

            static const uint8_t NUM_DSM_CHANNELS = 6;

            // IMU ------------------------------------------------------------------------

            static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
            static constexpr float GYRO_SCALE_FACTOR = 131.0;

            static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
            static constexpr float ACCEL_SCALE_FACTOR = 16384.0;

            MPU6050 _mpu6050;

            bfs::SbusRx _rx = bfs::SbusRx(&Serial2);

            // Motors --------------------------------------------------------------------

            const std::vector<uint8_t> MOTOR_PINS = { 3, 4, 5, 6 };

            OneShot125 _motors = OneShot125(MOTOR_PINS);

            uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;

            // Blinkenlights ---------------------------------------------------------
            static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
            static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;
            static const uint8_t LED_PIN = 0;
            hf::BlinkTask _blinkTask;

            // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
            static constexpr float PITCH_ROLL_PRESCALE = 30.0;    

            // Max yaw rate in deg/sec
            static constexpr float YAW_PRESCALE = 160.0;     

            // IMU calibration parameters
            static constexpr float ACC_ERROR_X = 0.0;
            static constexpr float ACC_ERROR_Y = 0.0;
            static constexpr float ACC_ERROR_Z = 0.0;
            static constexpr float GYRO_ERROR_X = 0.0;
            static constexpr float GYRO_ERROR_Y= 0.0;
            static constexpr float GYRO_ERROR_Z = 0.0;

            uint32_t _usec_curr;

            uint32_t _chan_1, _chan_2, _chan_3, _chan_4, _chan_5, _chan_6;

            // Safety
            bool _isArmed;
            bool _gotFailsafe;

            void readImu(
                    float & AccX, float & AccY, float & AccZ,
                    float & gyroX, float & gyroY, float & gyroZ
                    ) 
            {
                static float AccX_prev, AccY_prev, AccZ_prev;
                static float gyroX_prev, gyroY_prev, gyroZ_prev;

                int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

                _mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

                //Accelerometer
                AccX = AcX / ACCEL_SCALE_FACTOR; //G's
                AccY = AcY / ACCEL_SCALE_FACTOR;
                AccZ = AcZ / ACCEL_SCALE_FACTOR;

                // Correct the outputs with the calculated error values
                AccX = AccX - ACC_ERROR_X;
                AccY = AccY - ACC_ERROR_Y;
                AccZ = AccZ - ACC_ERROR_Z;

                // LP filter accelerometer data
                AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
                AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
                AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
                AccX_prev = AccX;
                AccY_prev = AccY;
                AccZ_prev = AccZ;

                // Gyro
                gyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
                gyroY = GyY / GYRO_SCALE_FACTOR;
                gyroZ = GyZ / GYRO_SCALE_FACTOR;

                // Correct the outputs with the calculated error values
                gyroX = gyroX - GYRO_ERROR_X;
                gyroY = gyroY - GYRO_ERROR_Y;
                gyroZ = gyroZ - GYRO_ERROR_Z;

                // LP filter gyro data
                gyroX = (1.0 - B_gyro)*gyroX_prev + B_gyro*gyroX;
                gyroY = (1.0 - B_gyro)*gyroY_prev + B_gyro*gyroY;
                gyroZ = (1.0 - B_gyro)*gyroZ_prev + B_gyro*gyroZ;
                gyroX_prev = gyroX;
                gyroY_prev = gyroY;
                gyroZ_prev = gyroZ;

                // Negate gyroZ for nose-right positive
                gyroZ = -gyroZ;
            }


            static void runLoopDelay(const uint32_t usec_curr)
            {
                //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
                /*
                 * It's good to operate at a constant loop rate for filters to remain
                 * stable and whatnot. Interrupt routines running in the background cause
                 * the loop rate to fluctuate. This function basically just waits at the
                 * end of every loop iteration until the correct time has passed since the
                 * start of the current loop for the desired loop rate in Hz. 2kHz is a
                 * good rate to be at because the loop nominally will run between 2.8kHz -
                 * 4.2kHz. This lets us have a little room to add extra computations and
                 * remain above 2kHz, without needing to retune all of our filtering
                 * parameters.
                 */
                float invFreq = 1.0/LOOP_FREQ_HZ*1000000.0;

                unsigned long checker = micros();

                //Sit in loop until appropriate time has passed
                while (invFreq > (checker - usec_curr)) {
                    checker = micros();
                }
            }

            static void blinkOnStartup(void)
            {
                // These constants are arbitrary, so we hide them here
                static const uint8_t  BLINK_INIT_COUNT = 10;
                static const uint32_t BLINK_INIT_TIME_MSEC = 100;

                for (uint8_t j=0; j<BLINK_INIT_COUNT; j++) {
                    digitalWrite(LED_PIN, LOW);
                    delay(BLINK_INIT_TIME_MSEC);
                    digitalWrite(LED_PIN, HIGH);
                    delay(BLINK_INIT_TIME_MSEC);
                }
            }

            void initImu() 
            {
                Wire.begin();
                Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

                _mpu6050.initialize();

                if (!_mpu6050.testConnection()) {
                    printf("MPU6050 initialization unsuccessful\n");
                    printf("Check MPU6050 wiring or try cycling power\n");
                    while(true) {}
                }

                // From the reset state all registers should be 0x00, so we should be at
                // max sample rate with digital low pass filter(s) off.  All we need to
                // do is set the desired fullscale ranges
                _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
                _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

            }

            void runMotors() 
            {
                _motors.set(0, _m1_usec);
                _motors.set(1, _m2_usec);
                _motors.set(2, _m3_usec);
                _motors.set(3, _m4_usec);

                _motors.run();
            }

            void cutMotors(const uint32_t chan_5, bool & isArmed) 
            {
                if (chan_5 < 1500 || !isArmed) {
                    isArmed = false;
                    _m1_usec = 120;
                    _m2_usec = 120;
                    _m3_usec = 120;
                    _m4_usec = 120;

                }
            }

            void readReceiver() 
            {
                if (_rx.Read()) {

                    const auto data = _rx.data();

                    if (data.lost_frame || data.failsafe) {

                        _isArmed = false;
                        _gotFailsafe = true;
                    }

                    else {
                        _chan_1 = data.ch[0];
                        _chan_2 = data.ch[1];
                        _chan_3 = data.ch[2];
                        _chan_4 = data.ch[3];
                        _chan_5 = data.ch[4];
                        _chan_6 = data.ch[5];
                    }
                }
            }

            static void armMotor(uint8_t & m_usec)
            {
                // OneShot125 range from 125 to 250 usec
                m_usec = 125;
            }

            static uint8_t scaleMotor(const float mval)
            {
                return hf::Utils::u8constrain(mval*125 + 125, 125, 250);

            }
    };

}
