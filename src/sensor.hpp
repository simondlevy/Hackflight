/*
   Sensor support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RoboFirmwareToolkit.hpp>
#include <RFT_sensor.hpp>

#include <state.hpp>

namespace hf {

    class Sensor : public rft::Sensor {

        protected:

            virtual void modifyState(rft::State * state, float time) override
            {
                modifyState((hf::State *)state, time);
            }

            virtual void modifyState(hf::State * state, float time) = 0;

    }; // class Sensor

} // namespace hf
