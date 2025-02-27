/*

   Main firmware support: IMU, receiver, motors

   Adapted from https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2025 Simon D. Levy

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

// Standard Arduino libraries
#include <Wire.h>
#include <SPI.h>

// Third-party libraries
#include <MPU6050.h>
#include <oneshot125.hpp>
#include <dsmrx.hpp>

// Hackflight library
#include <hackflight.hpp>
#include <estimators/madgwick.hpp>
#include <msp/serializer.hpp>
#include <rx.hpp>
#include <utils.hpp>
#include <timer.hpp>

static Dsm2048 _dsm2048;

// DSMX receiver callback
void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

namespace hf {

    class Board {

        // Settings we may want to adjust
        private:

            // FAFO -----------------------------------------------------------
            static const uint32_t LOOP_FREQ_HZ = 2000;

            // IMU ------------------------------------------------------------

            static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
            static constexpr float GYRO_SCALE_FACTOR = 131.0;

            static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
            static constexpr float ACCEL_SCALE_FACTOR = 16384.0;

            // LED
            static const uint8_t LED_PIN = 14;
            static constexpr float FAILSAFE_BLINK_RATE_HZ = 3;

            // Margins of safety for arming
            static constexpr float THROTTLE_ARMING_MAX = -0.9;
            static constexpr float ARMING_SWITCH_MIN = 0.9;

            // Motors ---------------------------------------------------------
            const std::vector<uint8_t> MOTOR_PINS = { 6, 5, 4, 3 };

            // Telemetry ------------------------------------------------------
            Timer _telemetryTimer = Timer(60); // Hz

            // Debugging ------------------------------------------------------
            Timer _debugTimer = Timer(100); // Hz

            // Raspberry Pi comms ---------------------------------------------
            Timer _rpiTimer = Timer(100); // Hz

            // Demand prescaling --------------------------------------------

            // Max pitch angle in degrees for angle mode (maximum ~70 degrees),
            // deg/sec for rate mode
            static constexpr float PITCH_ROLL_PRESCALE = 30.0;    

            // Max yaw rate in deg/sec
            static constexpr float YAW_PRESCALE = 160.0;     

            // IMU calibration parameters -------------------------------------

            static constexpr float ACC_ERROR_X = -0.030;
            static constexpr float ACC_ERROR_Y = +0.025;
            static constexpr float ACC_ERROR_Z = -0.11;
            static constexpr float GYRO_ERROR_X = -3.3;
            static constexpr float GYRO_ERROR_Y= -0.50;
            static constexpr float GYRO_ERROR_Z = -0.60;

        public:

            void init()
            {
                // Initialize the I^2C bus
                Wire.begin();

                // Initialize the I^2C sensors
                initImu();

                // Set up serial debugging
                Serial.begin(115200);

                // Set up serial connection from DSMX receiver
                Serial1.begin(115200);

                // Set up serial connection with Raspberry Pi
                Serial4.begin(115200);

                // Initialize the Madgwick filter
                _madgwick.initialize();

                // Arm OneShot125 motors
                _motors.arm();

                pinMode(LED_PIN, OUTPUT);
            }
 
            void readData(float & dt, demands_t & demands, state_t & state)
            {
                // Safety
                static bool _was_arming_switch_on;

                // Keep track of what time it is and how much time has elapsed
                // since the last loop
                _usec_curr = micros();      
                static uint32_t _usec_prev;
                dt = (_usec_curr - _usec_prev)/1000000.0;
                _usec_prev = _usec_curr;      

                // Read channels values from receiver
                if (_dsm2048.timedOut(micros())) {

                    _status = STATUS_FAILSAFE;
                }
                else if (_dsm2048.gotNewFrame()) {

                    _dsm2048.getChannelValuesMlp6Dsm(_channels);
                }

                // When throttle is down, toggle arming on switch press/release
                const auto is_arming_switch_on = _channels[5] > ARMING_SWITCH_MIN;

                if (_channels[0] < THROTTLE_ARMING_MAX &&
                        !is_arming_switch_on && _was_arming_switch_on) {
                    _status = 
                        _status == STATUS_READY ? STATUS_ARMED :
                        _status == STATUS_ARMED ? STATUS_READY :
                        _status;
                }
                _was_arming_switch_on = is_arming_switch_on;

                // Read IMU
                int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
                _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                // Accelerometer Gs
                const axis3_t accel = {
                    -ax / ACCEL_SCALE_FACTOR - ACC_ERROR_X,
                    ay / ACCEL_SCALE_FACTOR - ACC_ERROR_Y,
                    az / ACCEL_SCALE_FACTOR - ACC_ERROR_Z
                };

                // Gyro deg /sec
                const axis3_t gyro = {
                    gx / GYRO_SCALE_FACTOR - GYRO_ERROR_X, 
                    -gy / GYRO_SCALE_FACTOR - GYRO_ERROR_Y,
                    -gz / GYRO_SCALE_FACTOR - GYRO_ERROR_Z
                };

                // Run Madgwick filter to get get Euler angles from IMU values
                // (note negations)
                axis4_t quat = {};
                _madgwick.getQuaternion(dt, gyro, accel, quat);

                // Compute Euler angles from quaternion
                axis3_t angles = {};
                Utils::quat2euler(quat, angles);

                state.phi = angles.x;
                state.theta = angles.y;
                state.psi = angles.z;

                // Get angular velocities directly from gyro
                state.dphi = gyro.x;
                state.dtheta = -gyro.y;
                state.dpsi = gyro.z;

                // Convert stick demands to appropriate intervals with
                // appropriate signs
                demands.thrust = (_channels[0] + 1) / 2; // -1,+1 => 0,1
                demands.roll  = -_channels[1] * PITCH_ROLL_PRESCALE;
                demands.pitch = _channels[2] * PITCH_ROLL_PRESCALE;
                demands.yaw   = -_channels[3] * YAW_PRESCALE;

                // Use LED to indicate arming status
                digitalWrite(LED_PIN, _status == STATUS_ARMED ? HIGH : LOW);

                // Send state vector to Raspberry Pi periodically
                if (_rpiTimer.isReady(_usec_curr)) {
                    static MspSerializer _serializer;
                    _serializer.serializeFloats(122, (const float *)&state, 12);
                   Serial4.write(_serializer.payload, _serializer.payloadSize);
                }
           }

            void runMotors(const float * motors, const bool safeMode=true)
            {
                // Rescale motor values for OneShot125
                auto m1_usec = scaleMotor(motors[0]);
                auto m2_usec = scaleMotor(motors[1]);
                auto m3_usec = scaleMotor(motors[2]);
                auto m4_usec = scaleMotor(motors[3]);

                // Shut motors down when not armed
                if (safeMode && _status != STATUS_ARMED) {

                    m1_usec = 120;
                    m2_usec = 120;
                    m3_usec = 120;
                    m4_usec = 120;
                }

                // Run motors
                _motors.set(0, m1_usec);
                _motors.set(1, m2_usec);
                _motors.set(2, m3_usec);
                _motors.set(3, m4_usec);

                _motors.run();

                runLoopDelay(_usec_curr);
            }

            bool debugReady()
            {
                return _debugTimer.isReady(_usec_curr);
            }

        private:

            // Sensors
            MPU6050 _mpu6050;

            // Motors
            OneShot125 _motors = OneShot125(MOTOR_PINS);

            // Timing
            uint32_t _usec_curr;

            // Receiver
            float _channels[6];

            // State estimation
            MadgwickFilter  _madgwick;

            // Safety
            uint8_t _status;

            // Private methods -----------------------------------------------

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

            void initImu() 
            {
                //Note this is 2.5 times the spec sheet 400 kHz max...
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
            }

            static uint8_t scaleMotor(const float mval)
            {
                return Utils::u8constrain(mval*125 + 125, 125, 250);

            }

            static void reportForever(const char * message)
            {
                while (true) {
                    printf("%s\n", message);
                    delay(500);
                }

            }
    };

}
