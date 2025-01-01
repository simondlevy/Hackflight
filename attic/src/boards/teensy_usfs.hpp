/*

   Support for "turtle board" quadcopter

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

// Standard Arduino libraries
#include <Wire.h>
#include <SPI.h>

// Third-party libraries
#include <usfs.hpp>
#include <pmw3901.hpp>
#include <VL53L1X.h>
#include <oneshot125.hpp>

// Hackflight library
#include <hackflight.hpp>
#include <rx.hpp>
#include <utils.hpp>
#include <timer.hpp>

namespace hf {

    class Board {

        public:

            void init(Receiver & rx)
            {
                init(rx, false);
            }

            void readData(
                    float & dt,
                    Receiver & rx,
                    demands_t & demands,
                    state_t & state)
            {
                static state_t _state;

                // Keep track of what time it is and how much time has elapsed
                // since the last loop
                _usec_curr = micros();      
                static uint32_t _usec_prev;
                dt = (_usec_curr - _usec_prev)/1000000.0;
                _usec_prev = _usec_curr;      

                // Get vehicle commands for next loop iteration
                rx.read(_channels, _gotFailsafe);

                const auto isArmingSwitchOn = _channels[4] > 1500;

                // Arm vehicle if safe
                if (
                        !_gotFailsafe &&
                        isArmingSwitchOn &&
                        !_wasArmingSwitchOn &&
                        _channels[0] < 1050) {
                    _isArmed = true;
                }

                _wasArmingSwitchOn = isArmingSwitchOn;

                // LED should be on when armed
                if (_isArmed) {
                    digitalWrite(_ledPin, HIGH);
                }

                // Otherwise, blink LED as heartbeat or failsafe rate
                else {
                    blinkLed();
                }

                // Read IMU --------------------------------------------------

                const auto eventStatus = Usfs::checkStatus(); 

                if (Usfs::eventStatusIsError(eventStatus)) { 

                    Usfs::reportError(eventStatus);
                }

                if (Usfs::eventStatusIsQuaternion(eventStatus)) { 
                    float qw=0, qx=0, qy=0, qz=0;
                    _usfs.readQuaternion(qw, qx, qy, qz);
                    const axis4_t quat = {qw, qx, qy, qz};
                    axis3_t angles = {};
                    Utils::quat2euler(quat, angles);
                    _state.phi = angles.x;
                    _state.theta = angles.y;
                    _state.psi = angles.z;
                }

                if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

                    float gx=0, gy=0, gz=0;
                    _usfs.readGyrometerScaled(gx, gy, gz);
                    _state.dphi = gx;
                    _state.dtheta = -gy;
                    _state.dpsi = gz;
                }

                // -----------------------------------------------------------

                if (_flow) {

                    /*
                    // Read rangefinder, non-blocking
                    const uint16_t range = _vl53l1.read(false);

                    // Read optical flow sensor
                    int16_t flowDx = 0;
                    int16_t flowDy = 0;
                    bool gotFlow = false;
                    _pmw3901.readMotion(flowDx, flowDy, gotFlow); 
                    if (gotFlow) {
                        //printf("flow: %+03d  %+03d\n", flowDx, flowDy);
                    }*/
                }

                // Convert stick demands to appropriate intervals
                demands.thrust = mapchan(rx, _channels[0],  0.,  1.);
                demands.roll   = mapchan(rx, _channels[1], -1,  +1) *
                    PITCH_ROLL_PRESCALE;
                demands.pitch  = mapchan(rx, _channels[2], -1,  +1) *
                    PITCH_ROLL_PRESCALE;
                demands.yaw    = mapchan(rx, _channels[3], -1,  +1) *
                    YAW_PRESCALE;

                // Debug periodically as needed
                if (_debugTimer.isReady(_usec_curr, DEBUG_RATE_HZ)) {
                }

                // Output current state
                memcpy(&state, &_state, sizeof(state_t));
             }

            void runMotors(const float * motors)
            {
                // Rescale motor values for OneShot125
                auto m1_usec = scaleMotor(motors[0]);
                auto m2_usec = scaleMotor(motors[1]);
                auto m3_usec = scaleMotor(motors[2]);
                auto m4_usec = scaleMotor(motors[3]);

                // Turn off motors under various conditions
                if (_channels[4] < 1500 || !_isArmed || _gotFailsafe) {
                    _isArmed = false;
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

        private:

            // FAFO -----------------------------------------------------------
            static const uint32_t LOOP_FREQ_HZ = 2000;

            // USFS IMU ------------------------------------------------------

            static const uint8_t USFS_ACCEL_BANDWIDTH = 3;
            static const uint8_t USFS_GYRO_BANDWIDTH  = 3;
            static const uint8_t USFS_QUAT_DIVISOR    = 1;
            static const uint8_t USFS_MAG_RATE        = 100;
            static const uint8_t USFS_ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
            static const uint8_t USFS_GYRO_RATE       = 100; // Multiply by 10 to get actual rate
            static const uint8_t USFS_BARO_RATE       = 50;

            static const bool USFS_VERBOSE = false;

            static const uint8_t USFS_INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
                Usfs::INTERRUPT_ERROR |
                Usfs::INTERRUPT_QUAT;

            Usfs _usfs;

            // Rangefinder ----------------------------------------------------

            VL53L1X _vl53l1;

            // Optical flow sensor --------------------------------------------

            bool _flow;

            PMW3901 _pmw3901;

            // Motors ---------------------------------------------------------
            const std::vector<uint8_t> MOTOR_PINS = { 6, 5, 4, 3 };
            OneShot125 _motors = OneShot125(MOTOR_PINS);

            // Blinkenlights --------------------------------------------------
            static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
            static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;
            uint8_t _ledPin;

            // Debugging ------------------------------------------------------
            static constexpr float DEBUG_RATE_HZ = 100;
            Timer _debugTimer;

            // Demand prescaling --------------------------------------------

            // Max pitch angle in degrees for angle mode (maximum ~70 degrees),
            // deg/sec for rate mode
            static constexpr float PITCH_ROLL_PRESCALE = 30.0;    

            // Max yaw rate in deg/sec
            static constexpr float YAW_PRESCALE = 160.0;     

            // IMU calibration parameters -------------------------------------

            static constexpr float ACC_ERROR_X = 0.0;
            static constexpr float ACC_ERROR_Y = 0.0;
            static constexpr float ACC_ERROR_Z = 0.0;
            static constexpr float GYRO_ERROR_X = 0.0;
            static constexpr float GYRO_ERROR_Y= 0.0;
            static constexpr float GYRO_ERROR_Z = 0.0;

            uint32_t _usec_curr;

            uint16_t _channels[6];

            // Safety
            bool _isArmed;
            bool _gotFailsafe;
            bool _wasArmingSwitchOn;

            // Private methods -----------------------------------------------

            void init(Receiver & rx, bool flow)
            {
                _ledPin = flow ? 0 :LED_BUILTIN;

                _flow = flow;

                // Set up serial debugging
                Serial.begin(115200);

                // Start receiver
                rx.begin();

                // Initialize LED
                pinMode(_ledPin, OUTPUT); 
                digitalWrite(_ledPin, HIGH);

                // Initialize the I^2C bus
                Wire.begin();

                // Initialize the SPI bus if we're doing optical flow
                if (flow) {
                    SPI.begin();
                }

                // Initialize the sensors
                initImu();

                // Initial flow and rangefinder if indicated
                if (_flow) {
                    initRangefinder();
                    initOpticalFlow();
                }

                // Arm OneShot125 motors
                _motors.arm();

                _wasArmingSwitchOn = true;
            }

            static float mapchan(
                    Receiver & rx,
                    const uint16_t rawval,
                    const float newmin,
                    const float newmax)
            {
                return newmin + (rawval - (float)rx.minval()) / 
                    (rx.maxval() - (float)rx.minval()) * (newmax - newmin);
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

            void initImu() 
            {
                // Note this is 2.5 times the spec sheet 400 kHz max...
                Wire.setClock(1000000); 

                _usfs.reportChipId();        

                _usfs.loadFirmware(USFS_VERBOSE); 

                _usfs.begin(
                        USFS_ACCEL_BANDWIDTH,
                        USFS_GYRO_BANDWIDTH,
                        USFS_QUAT_DIVISOR,
                        USFS_MAG_RATE,
                        USFS_ACCEL_RATE,
                        USFS_GYRO_RATE,
                        USFS_BARO_RATE,
                        USFS_INTERRUPT_ENABLE,
                        USFS_VERBOSE); 

                // Clear interrupts
                Usfs::checkStatus();
            }

            void initRangefinder()
            {
                _vl53l1.setTimeout(500);

                if (!_vl53l1.init()) {
                    reportForever("VL53L1 initialization unsuccessful");
                }

                // Use long distance mode and allow up to 50000 us (50 ms) for
                // a measurement.  You can change these settings to adjust the
                // performance of the _vl53l1, but the minimum timing budget is
                // 20 ms for short distance mode and 33 ms for medium and long
                // distance modes. See the VL53L1X datasheet for more
                // information on range and timing limits.
                _vl53l1.setDistanceMode(VL53L1X::Long);
                _vl53l1.setMeasurementTimingBudget(50000);

                // Start continuous readings at a rate of one measurement every
                // 50 ms (the inter-measurement period). This period should be
                // at least as long as the timing budget.
                _vl53l1.startContinuous(50);
            }

            void initOpticalFlow()
            {
                if (!_pmw3901.begin()) {
                    reportForever("PMW3901 initialization unsuccessful");
                }
            }

            static uint8_t scaleMotor(const float mval)
            {
                return Utils::u8constrain(mval*125 + 125, 125, 250);

            }

            void blinkLed()
            {
                const auto freq_hz =
                    _gotFailsafe ?
                    FAILSAFE_BLINK_RATE_HZ :
                    HEARTBEAT_BLINK_RATE_HZ;

                static uint32_t _usec_prev;

                static uint32_t _delay_usec;

                if (_usec_curr - _usec_prev > _delay_usec) {

                    static bool _alternate;

                    _usec_prev = _usec_curr;

                    digitalWrite(_ledPin, _alternate);

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

            static void reportForever(const char * message)
            {
                while (true) {
                    printf("%s\n", message);
                    delay(500);
                }

            }
    };

}
