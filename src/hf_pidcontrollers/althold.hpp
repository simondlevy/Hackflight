/*
   PI controller for altitude hold

   Based on algorithm in
   http://ardupilot.org/copter/docs/altholdmode.html

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy


   MIT License
   */

#pragma once

#include "../HF_pidcontroller.hpp"

namespace hf {

    class AltitudeHoldPid : public PidController {

        private: 
 
            // Constants set in constructor ----------------

            float _Kp = 0;
            float _Ki = 0;
            float _windupMax = 0;
            float _stickDeadband = 0;
            float _pilotVelZMax = 0;

            // --------------------------------------------

        protected:

            void modifyDemands(state_t & state, demands_t & demands, bool ready) override
            {
                // Controller state ---------------------------
                static float errorI_;         
                static float altitudeTarget_; 
                static bool inBand_;  

                float altitude = state.z;

                // Is stick demand in deadband?
                bool inBand = fabs(demands.throttle) < _stickDeadband; 

                bool newInBand = inBand && !inBand_;

                inBand_ = ready ? inBand : inBand_;

                // Target velocity is a setpoint inside deadband, scaled
                // constant outside
                float targetVelocity = inBand ?  altitudeTarget_ - altitude
                    : _pilotVelZMax * demands.throttle;

                // Compute error as scaled target minus actual
                float error = targetVelocity - state.dz;

                // Compute I term, avoiding windup
                errorI_ = newInBand ? 0
                    : ready ? Filter::constrainAbs(errorI_ + error, _windupMax)
                    : errorI_;

                // Adjust throttle demand based on error
                demands.throttle = error * _Kp + errorI_ * _Ki;

                // If we re-entered deadband, we reset the target altitude.
                altitudeTarget_ = (ready && newInBand) ? altitude : altitudeTarget_;
            }

        public:

            AltitudeHoldPid(const float Kp = 0.75,
                    const float Ki = 1.5,
                    const float windupMax = 0.4,
                    const float pilotVelZMax = 2.5,
                    const float stickDeadband = 0.20)   
            {
                // Store constants
                _Kp = Kp;
                _Ki = Ki;
                _windupMax = windupMax;
                _pilotVelZMax = pilotVelZMax;
                _stickDeadband = stickDeadband;
            }

    };  // class AltitudeHoldPid

} // namespace hf
