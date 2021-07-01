/*
   Support for PWM3901 optical flow sensor

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

#include <RFT_sensor.hpp>
#include <RFT_filters.hpp>

#include <PMW3901.h>

namespace hf {

    class Pmw3901OpticalFlow : public rft::Sensor, public PMW3901 {

        friend class Hackflight;

        private:

            float _period = 0;
            float _time_prev = 0;

        protected:

            virtual void modifyState(rft::State * state, float time) override
            {
                // State * hfstate = (State *)state;

                int16_t deltaX = 0;
                int16_t deltaY = 0;

                PMW3901::readMotionCount(&deltaX, &deltaY);

                rft::Debugger::printf("%+04d  %+04d\n", deltaX, deltaY);
            }

            virtual void begin(void) override
            {
                PMW3901::begin();
            }

            virtual bool ready(float time) override
            {
                // Enough time must also elapse between readings
                if (time - _time_prev > _period) {
                    _time_prev = time;
                    return true;
                }

                return false;
            }

        public:

            Pmw3901OpticalFlow(uint8_t cspin, SPIClass * spi, uint16_t freq=50)
                : PMW3901(cspin, spi)
            {
                _period = 1. / freq;

                _time_prev = 0;
            }


    };  // class Pmw3901OpticalFlow 

} // namespace hf
