/*
   Hackflight "full monty" support (receiver, IMU, mixer, motors)

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "hackflightbase.hpp"
#include "imu.hpp"
#include "mspparser.hpp"
#include "mixer.hpp"
#include "motor.hpp"
#include "pidcontroller.hpp"
#include "timertasks/serialtask.hpp"
#include "sensors/surfacemount/gyrometer.hpp"
#include "sensors/surfacemount/quaternion.hpp"

namespace hf {

    class Hackflight : public HackflightBase {

        private: 

            // Passed to Hackflight::init() for a particular build
            IMU        * _imu      = NULL;
            Mixer      * _mixer    = NULL;

            // Serial timer task for GCS
            SerialTask _serialTask;

             // Mandatory sensors on the board
            Gyrometer _gyrometer;
            Quaternion _quaternion; // not really a sensor, but we treat it like one!

            void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
                float time = _board->getTime();

                // If quaternion data ready
                if (_quaternion.ready(time)) {

                    // Adjust quaternion values based on IMU orientation
                    _board->adjustQuaternion(_quaternion._w, _quaternion._x, _quaternion._y, _quaternion._z);

                    // Update state with new quaternion to yield Euler angles
                    _quaternion.modifyState(_state, time);

                    // Adjust Euler angles to compensate for sloppy IMU mounting
                    _board->adjustRollAndPitch(_state.rotation[0], _state.rotation[1]);
                }
            }

            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
                if (_gyrometer.ready(time)) {

                    // Adjust gyrometer values based on IMU orientation
                    _board->adjustGyrometer(_gyrometer._x, _gyrometer._y, _gyrometer._z);

                    // Update state with gyro rates
                    _gyrometer.modifyState(_state, time);

                    // Sync PID controllers to gyro update
                    runPidControllers();
                }
            }

        public:

            void init(Board * board, IMU * imu, Receiver * receiver, Mixer * mixer, Motor ** motors, bool armed=false)
            {  
                // Do general initialization
                HackflightBase::init(board, receiver, mixer);

                // Store pointers to IMU, mixer
                _imu   = imu;
                _mixer = mixer;

                // Initialize serial timer task
                _serialTask.init(board, &_state, mixer, receiver);

                // Support safety override by simulator
                _state.armed = armed;

                // Support for mandatory sensors
                add_sensor(&_quaternion, imu);
                add_sensor(&_gyrometer, imu);

                // Start the IMU
                imu->begin();

                // Tell the mixer which motors to use, and initialize them
                mixer->useMotors(motors);

            } // init

            void update(void)
            {
                // Grab control signal if available
                checkReceiver();

                // Check mandatory sensors
                checkGyrometer();
                checkQuaternion();

                // Check optional sensors
                checkOptionalSensors();

                // Update timer Tasks
                _serialTask.update();
            } 

    }; // class Hackflight

} // namespace
