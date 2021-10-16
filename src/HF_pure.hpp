/*
   Core Hackflight class

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

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

        public:

            HackflightPure(Mixer * mixer)
            {
                _mixer = mixer;
                _controller_count = 0;
            }

            void update(
                    uint32_t time_usec,
                    float rxtdmd, float rxrdmd, float rxpdmd,float rxydmd,
                    float state_phi, float state_theta, float state_psi,
                    float state_dphi, float state_dtheta, float state_dpsi,
                    bool pready,
                    float pidtdmd, float pidrdmd, float pidpdmd, float pidydmd,
                    motors_t & motors)
            {
                demands_t demands = {rxtdmd, pidrdmd, pidpdmd, pidydmd};

                // Use updated demands to run motors
                _mixer->run(demands, motors);

                _state.phi = state_phi;
                _state.theta = state_theta;
                _state.psi = state_psi;

                _state.dphi = state_dphi;
                _state.dtheta = state_dtheta;
                _state.dpsi = state_dpsi;
            }

            void addPidController(PidController * controller) 
            {
                _controllers[_controller_count++] = controller;
            }

    }; // class HackflightPure

} // namespace hf
