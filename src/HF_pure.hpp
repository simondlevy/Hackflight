/*
   Core Hackflight class

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_sensor.hpp"
#include "HF_timer.hpp"
#include "HF_pidcontroller.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"
#include "HF_motors.hpp"
#include "HF_debugger.hpp"

namespace hf {

    class HackflightPure {

        private:

            Timer _timer = Timer(300);

            // PID controllers
            PidController * _controllers[256] = {};
            uint8_t _controller_count = 0;

        protected:

            // Essentials
            Mixer * _mixer = NULL;
            state_t _state = {};

            // Sensors 
            Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

        public:

            HackflightPure(Mixer * mixer)
            {
                _mixer = mixer;
                _sensor_count = 0;
                _controller_count = 0;
            }

            void update(
                    uint32_t time_usec,
                    float tdmd, float rdmd, float pdmd,float ydmd,
                    float state_phi, float state_theta, float state_psi,
                    motors_t & motors)
            {
                // Start with demands from open-loop controller
                demands_t demands = {tdmd, rdmd, pdmd, ydmd};

                // Periodically apply PID controllers to demands
                bool ready = _timer.ready(time_usec);
                for (uint8_t k=0; k<_controller_count; ++k) {
                    _controllers[k]->modifyDemands(_state, demands, ready); 

                }

                // Use updated demands to run motors
                _mixer->run(demands, motors);

                // Run sensors
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState(_state, time_usec);
                }
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
