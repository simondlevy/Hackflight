/*
   Core Hackflight class

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_sensor.hpp"
#include "HF_timer.hpp"
#include "HF_pidcontroller.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"

namespace hf {

    class HackflightPure {

        private:

            Timer _timer = Timer(300);

            // PID controllers
            PidController * _controllers[256] = {};
            uint8_t _controller_count = 0;

            void checkSensors(uint32_t time_usec, State * state)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState((State *)state, time_usec);
                }
            }

        protected:

            // Essentials
            Receiver * _receiver = NULL;
            Mixer * _mixer = NULL;
            State _state = {};

            // Sensors 
            Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

        public:

            HackflightPure(Receiver * receiver, Mixer * mixer)
            {
                _receiver = receiver;
                _mixer = mixer;
                _sensor_count = 0;
                _controller_count = 0;
            }

            void begin(void)
            {  
                _state.armed = true;

            } // begin

            void update(uint32_t time_usec)
            {
                _receiver->update();

                // Start with demands from open-loop controller
                float demands[Receiver::MAX_DEMANDS] = {};
                _receiver->getDemands(demands);

                bool ready = _timer.ready(time_usec);

                // Apply PID controllers to demands
                for (uint8_t k=0; k<_controller_count; ++k) {
                    _controllers[k]->modifyDemands(_state.x, demands, ready); 

                }

                // Use updated demands to run motors, allowing mixer to choose
                // whether it cares about open-loop controller being inactive
                // (e.g., throttle down)
                if (!_state.failsafe && _state.armed && !_receiver->inactive()) {
                    _mixer->run(demands);
                }

                // Check sensors
                checkSensors(time_usec, &_state);
            }

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * controller) 
            {
                _controllers[_controller_count++] = controller;
            }

    }; // class HackflightPure

} // namespace hf
