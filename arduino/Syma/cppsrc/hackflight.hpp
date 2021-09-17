/*
   Hackflight algorithm for real vehicles

   Supports adding serial communications tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../copilot_extra.h"

#include "pidcontroller.hpp"
#include "receiver.hpp"
#include "sensor.hpp"
#include "mixer.hpp"
#include "serial.hpp"

namespace hf {

    class Hackflight {
        
        private:

            static constexpr float   LED_STARTUP_FLASH_SECONDS = 1.0;
            static constexpr uint8_t LED_STARTUP_FLASH_COUNT   = 20;

            Receiver * _receiver = NULL;
            Mixer * _mixer = NULL;

            uint8_t _led_pin = 0;

            State _state = {};
            SerialTask gcsTask = {};

            bool _safeToArm = false;
            Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

            // PID controllers
            PidController * _controllers[256] = {};
            uint8_t _controller_count = 0;

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
            }

            void checkSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState(&_state);
                }
            }

            void checkReceiver(float * motorsOut)
            {
                // Sync failsafe to open-loop-controller
                if (copilot_receiverLostSignal && _state.armed) {
                    _mixer->cut(motorsOut);
                    _state.armed = false;
                    _state.failsafe = true;
                    return;
                }

                // Update the receiver
                _receiver->update();

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
                    _mixer->cut(motorsOut);
                }

            } // checkReceiver

            void updatePidControllers(float * motorsOut)
            {
                // Start with demands from open-loop controller
                float demands[4] = {};
                _receiver->getDemands(demands);

                for (uint8_t k=0; k<_controller_count; ++k) {

                    PidController * controller = _controllers[k];

                    // Some controllers need to be reset based on inactivty
                    // (e.g., throttle down resets PID controller integral)
                    controller->resetOnInactivity(_receiver->inactive());

                    controller->modifyDemands(_state.x, demands); 
                }

                // Use updated demands to run motors, allowing
                // mixer to choose whether it cares about
                // open-loop controller being inactive (e.g.,
                // throttle down)
                if (!_state.failsafe) {
                    _mixer->run(demands,
                            _state.armed && !_receiver->inactive(),
                            motorsOut);
                }

             } // doTask

         public:

            Hackflight(Receiver * receiver, hf::Mixer * mixer, uint8_t ledPin)
            {
                _receiver = receiver;
                _mixer = mixer;
                _led_pin = ledPin;

                _sensor_count = 0;
                _controller_count = 0;
            }

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * controller)
            {
                _controllers[_controller_count++] = controller;
            }

            void begin(bool armed=false)
            {
                // Supports automatic arming for simulator
                _state.armed = armed;

                // Initialize the sensors
                startSensors();

                // Start the mixer
                _mixer->begin();
            }

            void update(float * motorsOut, bool & ledOut, serial_t & serial)
            {
                // Grab control signal if available
                checkReceiver(motorsOut);

                // Update PID controllers task
                updatePidControllers(motorsOut);

                // Check sensors
                checkSensors();

                // Update serial task
                gcsTask.parse(_mixer, &_state, serial);

                // Support motor testing from GCS
                if (!_state.armed) {
                    _mixer->runDisarmed(motorsOut);
                }

                // Determine LED state
                uint32_t time_msec = copilot_time_msec;
                ledOut = time_msec < 2000 ?  ((time_msec / 50) % 2) == 0 : _state.armed;

            }

    }; // class Hackflight

}  // namespace hf
