/*
   Support for VL53L1X time-of-flight distance sensor

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <RFT_sensor.hpp>

#include <VL53L1X.h>

namespace hf {

    class Vl53l1xRangefinder : public rft::Sensor {

        friend class Hackflight;

        private:

            VL53L1X _vl53l1x;

            // Support using temporal first different to compute State::DZ
            float _dist_prev = 0;
            float _time_prev = 0;

        protected:

            virtual void modifyState(rft::State * state, float time) override
            {
                State * hfstate = (State *)state;

                float dist = _vl53l1x.getDistance();

                hfstate->x[State::Z] = dist;

                if (_time_prev > 0) {
                    hfstate->x[State::DZ] = (dist - _dist_prev) / (time - _time_prev);
                }

                _dist_prev = dist;
                _time_prev = time;

            }

            virtual void begin(void) override
            {
                _vl53l1x.begin();
            }

            virtual bool ready(float time) override
            {
                return _vl53l1x.newDataReady();
            }

        public:

          Vl53l1xRangefinder(void)
          {
            _dist_prev = 0;
            _time_prev = 0;
          }


    };  // class Vl53l1xRangefinder 

} // namespace hf
