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

            void modifyDemands(float * state, float * demands, bool ready) override
            {
                // Controller state ---------------------------
                static float _errorI;         
                static float _altitudeTarget; 
                static bool _inBandPrev;  

                float altitude = state[State::Z];

                // Is stick demand in deadband?
                bool inBand = fabs(demands[DEMANDS_THROTTLE]) < _stickDeadband; 

                bool newInBand = inBand && !_inBandPrev;

                _inBandPrev = ready ? inBand : _inBandPrev;

                // Target velocity is a setpoint inside deadband, scaled
                // constant outside
                float targetVelocity = inBand ?
                    _altitudeTarget - altitude :
                    _pilotVelZMax *
                    demands[DEMANDS_THROTTLE];

                // Compute error as scaled target minus actual
                float error = targetVelocity - state[State::DZ];

                // Compute I term, avoiding windup
                _errorI = newInBand ? 0
                    : ready ? Filter::constrainAbs(_errorI + error, _windupMax)
                    : _errorI;

                // Adjust throttle demand based on error
                demands[DEMANDS_THROTTLE] = error * _Kp + _errorI * _Ki;

                // If we re-entered deadband, we reset the target altitude.
                _altitudeTarget = (ready && newInBand) ? altitude : _altitudeTarget;
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
