/*
   PI controller for altitude hold

   Based on algorithm in
   http://ardupilot.org/copter/docs/altholdmode.html

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy


   MIT License
   */

#pragma once

#include "HF_pidcontroller.hpp"

namespace hf {

    class AltitudeHoldPid : public PidController {

        private: 
 
            // Constants set in instructor ----------------

            float _Kp = 0;
            float _Ki = 0;
            float _windupMax = 0;
            float _stickDeadband = 0;
            float _pilotVelZMax = 0;

            // Controller state ---------------------------

            // Error integral
            float _errorI = 0;

            // Will be reset each time we re-enter deadband
            float _altitudeTarget = 0;

            // Tracks whether we just entered deadband
            bool _inBandPrev = false;

            // --------------------------------------------

        protected:

            void modifyDemands(float * state, float * demands) override
            {
                bool didReset = false;
                float altitude = state[State::Z];

                // Is stick demand in deadband?
                bool inBand = fabs(demands[DEMANDS_THROTTLE]) <
                    _stickDeadband; 

                // Reset controller when moving into deadband
                if (inBand && !_inBandPrev) {
                    _errorI = 0;
                    didReset = true;
                }
                _inBandPrev = inBand;

                // Target velocity is a setpoint inside deadband, scaled
                // constant outside
                float targetVelocity = inBand ?
                                       _altitudeTarget - altitude :
                                       _pilotVelZMax *
                                       demands[DEMANDS_THROTTLE];

                // Compute error as scaled target minus actual
                float error = targetVelocity - state[State::DZ];

                // Compute I term, avoiding windup
                _errorI = rft::Filter::constrainAbs(_errorI + error, _windupMax);

                // Adjust throttle demand based on error
                demands[DEMANDS_THROTTLE] = error * _Kp + _errorI * _Ki;

                // If we re-entered deadband, we reset the target altitude.
                if (didReset) {
                    _altitudeTarget = altitude;
                }
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
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

                // Initialize state
                _errorI = 0;
                _inBandPrev = false;
                _altitudeTarget = 0;
            }

    };  // class AltitudeHoldPid

} // namespace hf
