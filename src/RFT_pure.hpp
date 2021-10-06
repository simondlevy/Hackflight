/*
   Core class for RoboFirmwareToolkit

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_board.hpp"
#include "RFT_openloop.hpp"
#include "RFT_closedloop.hpp"
#include "RFT_sensor.hpp"
#include "RFT_actuator.hpp"
#include "RFT_parser.hpp"
#include "RFT_closedlooptask.hpp"

namespace rft {

    class RFTPure {

        private:

            // Safety
            bool _safeToArm = false;

            // Sensors 
            Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

            // Timer task for PID controllers
            ClosedLoopTask _closedLoopTask;

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
            }

            void checkSensors(State * state)
            {
                // Some sensors may need to know the current time
                float time = _board->getTime();

                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState(state, time);
                }
            }

            void checkOpenLoopController(State * state)
            {
                // Sync failsafe to open-loop-controller
                if (_olc->lostSignal() && state->armed) {
                    _actuator->cut();
                    state->armed = false;
                    state->failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether controller data is available
                if (!_olc->ready()) return;

                // Disarm
                if (state->armed && !_olc->inArmedState()) {
                    state->armed = false;
                } 

                // Avoid arming when controller is in armed state
                if (!_safeToArm) {
                    _safeToArm = !_olc->inArmedState();
                }

                // Arm after lots of safety checks
                if (_safeToArm
                    && !state->armed
                    && !state->failsafe 
                    && state->safeToArm()
                    && _olc->inactive()
                    && _olc->inArmedState()
                    ) {
                    state->armed = true;
                }

                // Cut motors on inactivity
                if (state->armed && _olc->inactive()) {
                    _actuator->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(state->armed);

            } // checkOpenLoopController

        protected:

            // Essentials
            Board * _board = NULL;
            OpenLoopController * _olc = NULL;
            Actuator * _actuator = NULL;

            RFTPure(Board * board,
                    OpenLoopController * olc,
                    Actuator * actuator)
            {
                _board = board;
                _olc = olc;
                _actuator = actuator;

                _sensor_count = 0;
            }

            void begin(void)
            {  
                // Start the board
                _board->begin();

                // Initialize the sensors
                startSensors();

                // Initialize the open-loop controller
                _olc->begin();

                // Start the actuator
                _actuator->begin();

            } // begin

            void update(State * state)
            {
                // Grab control signal if available
                checkOpenLoopController(state);

                // Update PID controllers task
                _closedLoopTask.update(_board, _olc, _actuator, state);

                // Check sensors
                checkSensors(state);
            }

        public:

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addClosedLoopController(ClosedLoopController * controller,
                                         uint8_t modeIndex=0) 
            {
                _closedLoopTask.addController(controller, modeIndex);
            }

    }; // class RFTPure

} // namespace rft
