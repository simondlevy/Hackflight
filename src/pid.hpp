#pragma once

#include "num.hpp"
#include "lpf.hpp"

class Pid {

    private:

        static constexpr float DEFAULT_PID_INTEGRATION_LIMIT = 5000;
        static constexpr float DEFAULT_PID_OUTPUT_LIMIT =  0;

        float _desired;      // set point
        float _error;        // error
        float _prevError;    // previous error
        float _integ;        // integral
        float _deriv;        // derivative
        float _kp;           // proportional gain
        float _ki;           // integral gain
        float _kd;           // derivative gain
        float _kff;          // feedforward gain
        float _outP;         // proportional output (debugging)
        float _outI;         // integral output (debugging)
        float _outD;         // derivative output (debugging)
        float _outFF;        // feedforward output (debugging)
        float _iLimit;       // integral limit, absolute value. '0' means no limit.
        float _outputLimit;  // total PID output limit, absolute value. '0' means no limit.
        float _dt;           // delta-time dt
        Lpf _dFilter;        // filter for D term
        bool _enableDFilter; // filter for D term enable flag

    public:

        void init(
                const float kp,
                const float ki,
                const float kd,
                const float kff,
                const float dt,
                const float samplingRate,
                const float cutoffFreq,
                bool enableDFilter)
        {
            _error         = 0;
            _prevError     = 0;
            _integ         = 0;
            _deriv         = 0;
            _desired       = 0;
            _kp            = kp;
            _ki            = ki;
            _kd            = kd;
            _kff           = kff;
            _iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
            _outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
            _dt            = dt;
            _enableDFilter = enableDFilter;

            if (_enableDFilter) {
                _dFilter.init(samplingRate, cutoffFreq);
            }
        }

        void setIntegralLimit(const float limit)
        {
            _iLimit = limit;
        }

        void reset(void)
        {
            _error     = 0;
            _prevError = 0;
            _integ     = 0;
            _deriv     = 0;
        }

        float run(const float desired, const float measured)
        {
            _desired = desired;

            _error = _desired - measured;

            return run();
        }

        float run(void)
        {
            _outP = _kp * _error;

            auto output = _outP;

            float deriv = (_error - _prevError) / _dt;

            if (_enableDFilter){
                _deriv = _dFilter.apply(deriv);
            } else {
                _deriv = deriv;
            }

            if (isnan(_deriv) || isinf(_deriv)) {
                _deriv = 0;
            }

            _outD = _kd * _deriv;
            output += _outD;

            _integ += _error * _dt;

            // Constrain the integral (unless the iLimit is zero)
            if(_iLimit != 0) {
                _integ = Num::fconstrain(_integ, -_iLimit, _iLimit);
            }

            _outI = _ki * _integ;
            output += _outI;

            _outFF = _kff * _desired;
            output += _outFF;

            // Constrain the total PID output (unless the outputLimit is zero)
            if(_outputLimit != 0) {
                output = Num::fconstrain(output, -_outputLimit, _outputLimit);
            }

            _prevError = _error;

            return output;
        }

        void setDesired(const float desired)
        {
            _desired = desired;
        }

        void setError(const float error)
        {
            _error = error;
        }

        void setOutputLimit(const float outputLimit)
        {
            _outputLimit = outputLimit;
        }

        void filterReset(const float samplingRate, const float cutoffFreq, 
                bool enableDFilter) 
        {
            _enableDFilter = enableDFilter;

            if (_enableDFilter)
            {
                _dFilter.init(samplingRate, cutoffFreq);
            }
        }

}; // class Pid
