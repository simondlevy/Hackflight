/*
   Copyright (C) 2024 Simon D. Levy

   Based in part on https://github.com/nickrehm/dRehmFlight

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
#include <control.hpp>
#include <utils.hpp>
#include <timer.hpp>
#include <estimators/madgwick.hpp>
#include <mixers/bfquadx.hpp>
#include <espnow/listener.hpp>
#include <espnow/utils.hpp>
#include <msp.hpp>

namespace hf {

    class Board : public EspNowListener {

        private:

            // Blinkenlights -------------------------------------------------------------
            static const uint32_t LED_FAILSAFE_COLOR = 0xFF0000;
            static const uint32_t LED_HEARTBEAT_COLOR = 0x00FF00;
            static const uint32_t LED_ARMED_COLOR = 0xFF0000;
            static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
            static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;
            TinyPICO _tinypico;

            // Motors --------------------------------------------------------------------
            const std::vector<uint8_t> MOTOR_PINS = { 23, 26, 27, 15 };
            OneShot125 _motors = OneShot125(MOTOR_PINS);
            uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;

            // Receiver ------------------------------------------------------------------
            static const uint16_t CHAN5_ARM_MIN = 1000;
            static const uint16_t CHAN1_ARM_MAX = 180;
            uint16_t _channels[6];

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

            // -----------------------------------------------------
            static constexpr uint8_t TRANSMITTER_ADDRESS[6] = {
                0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4
            };

            // Telemetry ------------------------------------------------------
            static constexpr float TELEMETRY_RATE_HZ = 60;
            static constexpr uint8_t TELEMETRY_DONGLE_ADDRESS[6] = {
                0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4
            };
            Timer _telemetryTimer;

            // Prescaling for controx axes ------------------------------------
            static constexpr float PITCH_ROLL_PRESCALE = 30.0;    
            static constexpr float YAW_PRESCALE = 160.0;     

            // Debugging -----------------------------------------------------
            static constexpr float DEBUG_RATE_HZ = 100;

            // FAFO ----------------------------------------------------------
            static const uint32_t LOOP_FREQ_HZ = 2000;

            // Failsafe -------------------------------------------------------
            static const int32_t FAILSAFE_TIMEOUT_MSEC = 1000;
            uint32_t _last_received_msec;

            // Sensor fusion --------------------------------------------------
            MadgwickFilter _madgwick;

            // Utils ---------------------------------------------------------

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

            static float mapchan(
                    const uint16_t rawval,
                    const float newmin,
                    const float newmax)
            {
                return newmin + (rawval - (float)172) / 
                    (1811 - (float)172) * (newmax - newmin);
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

            static void armMotor(uint8_t & m_usec)
            {
                // OneShot125 range from 125 to 250 usec
                m_usec = 125;
            }

            static uint8_t scaleMotor(const float mval)
            {
                return Utils::u8constrain(mval*125 + 125, 125, 250);

            }

            static void on_espnow_data_receive(
                    const uint8_t * mac, const uint8_t * data, int len)
            {
                (void)mac;
            }

        public: // -----------------------------------------------------------

            void init() 
            {
                // Start serial debugging
                Serial.begin(115200);

                // Start ESP-NOW
                EspNowUtils::init();

                // Add the telemetry dongle as a peer
                EspNowUtils::addPeer(TELEMETRY_DONGLE_ADDRESS);

                // Add the transmitter as a peer
                EspNowUtils::addPeer(TRANSMITTER_ADDRESS);

                // Register an ESP-NOW callback for data sent by the
                // transmitter
                EspNowUtils::set_listener_callback(this);

                // Start I^2
                Wire.begin();

                // Note this is 2.5 times the spec sheet 400 kHz max...
                Wire.setClock(1000000); 

                // Start the IMU
                _mpu6050.initialize();

                if (!_mpu6050.testConnection()) {
                    reportForever("MPU6050 initialization unsuccessful\n");
                }

                // From the reset state all registers should be 0x00, so we
                // should be at max sample rate with digital low pass filter(s)
                // off.  All we need to do is set the desired fullscale ranges
                _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
                _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

                // Initialize the Madgwick filter
                _madgwick.initialize();

                // Arm motors
                armMotor(_m4_usec);
                armMotor(_m2_usec);
                armMotor(_m1_usec);
                armMotor(_m3_usec);
                _motors.arm();
            }

            void step(Control * control) 
            {
                static bool _isArmed;
                static bool _gotFailsafe;
                static uint32_t _usec_prev;
                static Timer _debugTimer;
                static BfQuadXMixer _mixer;
                static state_t _state;

                const auto usec_curr = micros();      

                const float dt = (usec_curr - _usec_prev)/1000000.0;

                _usec_prev = usec_curr;

                if ((int32_t)millis() -_last_received_msec >
                        FAILSAFE_TIMEOUT_MSEC) {
                    _gotFailsafe = true;
                }

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

                // Populate vehicle state with Euler angles and gyro values
                _state.phi = angles.x;
                _state.theta = angles.y;
                _state.psi = angles.z;
                _state.dphi = gyro.x;
                _state.dtheta = -gyro.y;
                _state.dpsi = gyro.z;

                // Convert stick demands to appropriate intervals
                demands_t demands = {
                    mapchan(_channels[0],  0.,  1.),
                    mapchan(_channels[1], -1,  +1) * PITCH_ROLL_PRESCALE,
                    mapchan(_channels[2], -1,  +1) * PITCH_ROLL_PRESCALE,
                    mapchan(_channels[3], -1,  +1) * YAW_PRESCALE
                };

                // Debug periodically as needed
                if (_debugTimer.isReady(usec_curr, DEBUG_RATE_HZ)) {
                }

                // Run closed-loop control
                control->run(dt, _state, demands);

                // Mix motors
                float motors[4] = {};
                _mixer.run(demands, motors);

                // Rescale motor values for OneShot125
                _m1_usec = scaleMotor(motors[0]);
                _m2_usec = scaleMotor(motors[1]);
                _m3_usec = scaleMotor(motors[2]);
                _m4_usec = scaleMotor(motors[3]);

                // Turn off motors under various conditions
                cutMotors(_channels[4], _isArmed); 

                // Run motors
                runMotors(); 

                // LED should be on when armed
                if (_isArmed) {
                    _tinypico.DotStar_SetPixelColor(LED_ARMED_COLOR);
                }

                // Otherwise, blink LED as heartbeat or failsafe rate
                else {
                    blinkLed(usec_curr, _gotFailsafe);
                }

                // Runn telemetry
                if (_telemetryTimer.isReady(usec_curr, TELEMETRY_RATE_HZ)) {

                    const float vals[10] = {
                        _state.dx, _state.dy, _state.z, _state.dz, _state.phi,
                        _state.dphi, _state.theta, _state.dtheta, _state.psi,
                        _state.dpsi
                    };

                    static Msp _msp;

                    _msp.serializeFloats(121, vals, 10);

                    EspNowUtils::sendToPeer(
                            TELEMETRY_DONGLE_ADDRESS,
                            _msp.payload,_msp.payloadSize,
                            "fc",
                            "dongle");
                }
 
                // Run dRehmFlight loop delay
                runLoopDelay(usec_curr);
            }

            void espnow_listener_callback(const uint8_t * data, const uint8_t len)
            {
                static Msp _msp;

                for (uint8_t k=0; k<len; ++k) {

                    if (_msp.parse(data[k]) == 200) {

                        _last_received_msec = millis();

                        readChannel(_msp, 0);
                        readChannel(_msp, 1);
                        readChannel(_msp, 2);
                        readChannel(_msp, 3);
                        readChannel(_msp, 4);
                        readChannel(_msp, 5);
                    }
                }
            }

            void readChannel(const Msp & msp, const uint8_t j)
            {
                _channels[j] = msp.payload[2*j+1] << 8 | msp.payload[2*j];
            }

    }; // class Board

} // namespace hf
