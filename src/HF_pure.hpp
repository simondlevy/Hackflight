/*
   Core Hackflight class

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_sensor.hpp"
#include "HF_pidtask.hpp"
#include "HF_pidcontroller.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"

namespace hf {

    class HackflightPure {

        private:

            // Timer task for PID controllers
            PidTask _pidTask;

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
            }

            void begin(void)
            {  
                _state.armed = true;

            } // begin

            void update(uint32_t time_usec)
            {
                _receiver->update();

                // Update PID controllers task
                _pidTask.update(time_usec, _receiver, _mixer, &_state);

                // Check sensors
                checkSensors(time_usec, &_state);
            }

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * controller) 
            {
                _pidTask.addController(controller);
            }

    }; // class HackflightPure

} // namespace hf
