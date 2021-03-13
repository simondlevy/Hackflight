/*
   Spektrum DSMX support for Arduino flight controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "receiver.hpp"
#include <DSMRX.h>

namespace hf {

    class DSMX_Receiver : public Receiver {

        private:

            DSM2048 _rx;

        protected:

            void begin(void)
            {
            }

            bool gotNewFrame(void)
            {
                return _rx.gotNewFrame();
            }

            void readRawvals(void)
            {
                _rx.getChannelValuesNormalized(rawvals, MAXCHAN);
            }

            bool lostSignal(void)
            {
                return _rx.timedOut(micros());
            }

        public:

            DSMX_Receiver(const uint8_t channelMap[6], const float demandScale)
                :  Receiver(channelMap, demandScale) 
            { 
            }

            void handleSerialEvent(uint8_t value, uint32_t usec)
            {
                _rx.handleSerialEvent(value, usec);
            }

    }; // class DSMX_Receiver

} // namespace hf
