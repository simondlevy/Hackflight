/*
   Support for VL53L1X time-of-flight distance sensor

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <RFT_sensor.hpp>

namespace hf {

    class VL53L1X : public rft::Sensor {

        friend class Hackflight;

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

    };  // class VL523L1X 

} // namespace hf
