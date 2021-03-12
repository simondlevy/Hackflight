/*
   Spektrum DSMX support for Arduino flight controllers using Serial1

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "openloops/receivers/arduino/dsmx.hpp"
#include <DSMRX.h>

static hf::DSMX_Receiver * _dsmx_rx;

void serialEvent1(void)
{
    while (Serial1.available()) {

        _dsmx_rx->handleSerialEvent(Serial1.read(), micros());
    }
}

namespace hf {

    class DSMX_Receiver_Serial1 : public DSMX_Receiver {

        protected:

            void begin(void) override 
            {
                Receiver::begin();

                Serial1.begin(115200);
            }

        public:

            DSMX_Receiver_Serial1(const uint8_t channelMap[6], const float demandScale)
                :  DSMX_Receiver(channelMap, demandScale) 
            { 
                _dsmx_rx = this;
            }

    }; // class DSMX_Receiver_Serial1

} // namespace hf
