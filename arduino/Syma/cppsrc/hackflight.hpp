/*
   Hackflight algorithm for real vehicles

   Supports adding serial communications tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "board.hpp"
#include "openloop.hpp"
#include "pidcontroller.hpp"
#include "receiver.hpp"
#include "sensor.hpp"
#include "mixer.hpp"
#include "parser.hpp"
#include "pidtask.hpp"
#include "serialtask.hpp"

namespace hf {

    class Hackflight {
        
        private:

            Board * _board = NULL;
            Receiver * _receiver = NULL;
            Mixer * _mixer = NULL;

            State _state = {};
            SerialTask * _serial_tasks[10] = {};
            uint8_t _serial_task_count = 0;

            bool _safeToArm = false;
            Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

            // Timer task for PID controllers
            PidControlTask _closedLoopTask;

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
            }

            void checkSensors(void)
            {
                // Some sensors may need to know the current time
                float time = _board->getTime();

                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState(&_state, time);
                }
            }

            void checkReceiver(void)
            {
                // Sync failsafe to open-loop-controller
                if (_receiver->lostSignal() && _state.armed) {
                    _mixer->cut();
                    _state.armed = false;
                    _state.failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether controller data is available
                if (!_receiver->ready()) return;

                // Disarm
                if (_state.armed && !_receiver->inArmedState()) {
                    _state.armed = false;
                } 

                // Avoid arming when controller is in armed state
                if (!_safeToArm) {
                    _safeToArm = !_receiver->inArmedState();
                }

                // Arm after lots of safety checks
                if (_safeToArm
                    && !_state.armed
                    && !_state.failsafe 
                    && _state.safeToArm()
                    && _receiver->inactive()
                    && _receiver->inArmedState()
                    ) {
                    _state.armed = true;
                }

                // Cut motors on inactivity
                if (_state.armed && _receiver->inactive()) {
                    _mixer->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

         public:

            Hackflight( hf::Board * board, Receiver * receiver, hf::Mixer * mixer)
            {
                _board = board;
                _receiver = receiver;
                _mixer = mixer;

                _sensor_count = 0;
                _serial_task_count = 0;
            }

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * controller,
                                         uint8_t modeIndex=0) 
            {
                _closedLoopTask.addController(controller, modeIndex);
            }

            void begin(bool armed=false)
            {
                // Supports automatic arming for simulator
                _state.armed = armed;

                // Start the board
                _board->begin();

                // Initialize the sensors
                startSensors();

                // Initialize the open-loop controller
                _receiver->begin();

                // Start the mixer
                _mixer->begin();
            }

            void update(void)
            {
                // Grab control signal if available
                checkReceiver();

                // Update PID controllers task
                _closedLoopTask.update(_board, _receiver, _mixer, &_state);

                // Check sensors
                checkSensors();

                // Update serial tasks
                for (uint8_t k=0; k<_serial_task_count; ++k) {
                    _serial_tasks[k]->update(_board, _mixer, &_state);
                }
            }

            void addSerialTask(SerialTask * task)
            {
                task->init(_mixer, &_state);
                _serial_tasks[_serial_task_count++] = task;
            }

    }; // class Hackflight

}  // namespace hf
