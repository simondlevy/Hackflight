/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "debugger.hpp"
#include "mspparser.hpp"
#include "imu.hpp"
#include "board.hpp"
#include "actuator.hpp"
#include "receiver.hpp"
#include "state.hpp"
#include "pidcontroller.hpp"
#include "motor.hpp"
#include "actuator.hpp"
#include "sensors/surfacemount.hpp"
#include "timertasks/pidtask.hpp"
#include "timertasks/serialtask.hpp"
#include "sensors/surfacemount/gyrometer.hpp"
#include "sensors/surfacemount/quaternion.hpp"

namespace hf {

    class Hackflight {

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

            // Supports periodic ad-hoc debugging
            Debugger _debugger;

            // Actuator
            Actuator * _actuator = NULL;

            // Sensors 
            Sensor * _sensors[256] = {NULL};
            uint8_t _sensor_count = 0;

            // Safety
            bool _safeToArm = false;

            // Support for headless mode
            float _yawInitial = 0;

            // Timer task for PID controllers
            PidTask _pidTask;

            // Passed to Hackflight::begin() for a particular build
            IMU * _imu      = NULL;

            // Serial timer task for GCS
            SerialTask _serialTask;

             // Mandatory sensors on the board
            Gyrometer _gyrometer;
            Quaternion _quaternion; // not really a sensor, but we treat it like one!
 
            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.rotation[axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

           void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
                float time = _board->getTime();

                // If quaternion data ready
                if (_quaternion.ready(time)) {

                    // Update state with new quaternion to yield Euler angles
                    _quaternion.modifyState(_state, time);
                }
            }

            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
                if (_gyrometer.ready(time)) {

                    // Update state with gyro rates
                    _gyrometer.modifyState(_state, time);
                }
            }


            Board    * _board    = NULL;
            Receiver * _receiver = NULL;

            // Vehicle state
            state_t _state;

            void checkOptionalSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(_state, time);
                    }
                }
            }

            void add_sensor(Sensor * sensor)
            {
                _sensors[_sensor_count++] = sensor;
            }

            void add_sensor(SurfaceMountSensor * sensor, IMU * imu) 
            {
                add_sensor(sensor);

                sensor->imu = imu;
            }

            void checkReceiver(void)
            {
                // Sync failsafe to receiver
                if (_receiver->lostSignal() && _state.armed) {
                    _actuator->cut();
                    _state.armed = false;
                    _state.failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether receiver data is available
                if (!_receiver->getDemands(_state.rotation[AXIS_YAW] - _yawInitial)) return;

                // Disarm
                if (_state.armed && !_receiver->getAux1State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux1 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux1State();
                }

                // Arm (after lots of safety checks!)
                if (_safeToArm && !_state.armed && _receiver->throttleIsDown() && _receiver->getAux1State() && 
                        !_state.failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.rotation[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _actuator->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver


        public:

            Hackflight(Board * board, IMU * imu, Receiver * receiver, Actuator * actuator)
            {  
                // Store the essentials
                _board = board;
                _receiver = receiver;
                _actuator = actuator;
                _imu = imu;
            }

            void begin(bool armed=false)
            {  
                _board->begin();

                // Ad-hoc debugging support
                _debugger.begin(_board);

                // Support adding new sensors and PID controllers
                _sensor_count = 0;

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Initialize the receiver
                _receiver->begin();

                // Setup failsafe
                _state.failsafe = false;

                // Initialize timer task for PID controllers
                _pidTask.begin(_board, _receiver, _actuator, &_state);
 
                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _receiver, _actuator);

                // Support safety override by simulator
                _state.armed = armed;

                // Support for mandatory sensors
                add_sensor(&_quaternion, _imu);
                add_sensor(&_gyrometer, _imu);

                // Start the IMU
                _imu->begin();

                // Tell the actuator to start the motors
                _actuator->begin();

            } // begin

            void addSensor(Sensor * sensor) 
            {
                add_sensor(sensor);
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                _pidTask.addPidController(pidController, auxState);
            }

            void update(void)
            {
                // Grab control signal if available
                checkReceiver();

                // Update PID controllers task
                _pidTask.update();

                // Check mandatory sensors
                checkGyrometer();
                checkQuaternion();

                // Check optional sensors
                checkOptionalSensors();

                // Update serial comms task
                _serialTask.update();
            }

    }; // class Hackflight

} // namespace
