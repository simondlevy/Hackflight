/*
   Support for VL53L1X time-of-flight distance sensor

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <RFT_sensor.hpp>
#include <RFT_filters.hpp>

#include <VL53L1X.h>

namespace hf {

    class Vl53l1xRangefinder : public rft::Sensor {

        friend class Hackflight;

        private:

            VL53L1X _vl53l1x;

            rft::LowPassFilter _lpf;

            float _period = 0;

            // Support using temporal first different to compute State::DZ
            float _dist_prev = 0;
            float _time_prev = 0;

        protected:

            virtual void modifyState(rft::State * state, float time) override
            {
                /*
                State * hfstate = (State *)state;

                float dist = _vl53l1x.getDistance(); // mm

                //hfstate->x[State::Z] = dist;

                if (_time_prev > 0) {

                    float dz = dist - _dist_prev;
                    float dt = time - _time_prev;

                    //hfstate->x[State::DZ] = (dist - _dist_prev) / dt;

                    rft::Debugger::printf("%f   %f\n", dist, _dist_prev);
                }

                _dist_prev = dist;
                */
            }

            virtual void begin(void) override
            {
                _vl53l1x.begin();

                _lpf.begin();
            }

            virtual bool ready(float time) override
            {
                // Sensor must have new data
                if (!_vl53l1x.newDataReady()) {
                    return false;
                }

                // Enough time must also elapse between readings
                if (time - _time_prev > _period) {
                    rft::Debugger::printf("ready\n");
                    _time_prev = time;
                    return true;
                }

                return false;
            }

        public:

            Vl53l1xRangefinder(uint16_t freq=2)
            {

                _period = 1. / freq;

                _dist_prev = 0;
                _time_prev = 0;
            }


    };  // class Vl53l1xRangefinder 

} // namespace hf
