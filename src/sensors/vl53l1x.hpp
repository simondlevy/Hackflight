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

            VL53L1X vl53l1x;

        protected:

            virtual void modifyState(rft::State * state, float time) override
            {
                (void)time;

                State * hfstate = (State *)state;

            }

            virtual bool ready(float time) override
            {
                return false;
            }

    };  // class Vl53l1xRangefinder 

} // namespace hf
