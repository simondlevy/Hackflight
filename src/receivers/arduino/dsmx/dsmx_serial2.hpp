/*
   Spektrum DSMX support for Arduino flight controllers using Serial2

   Copyright (c) 2019 Simon D. Levy

   MIT License
 */

#pragma once

#include "receivers/arduino/dsmx.hpp"
#include <DSMRX.h>

static hf::DSMX_Receiver * _dsmx_rx;

void serialEvent2(void)
{
    while (Serial2.available()) {

        _dsmx_rx->handleSerialEvent(Serial2.read(), micros());
    }
}

namespace hf {

    class DSMX_Receiver_Serial2 : public DSMX_Receiver {

        //protected:
        public:

            void begin(void) override 
            {
                Receiver::begin();

                Serial2.begin(115200);
            }

        public:

            DSMX_Receiver_Serial2(const uint8_t channelMap[6], const float demandScale)
                :  DSMX_Receiver(channelMap, demandScale) 
            { 
                _dsmx_rx = this;
            }

    }; // class DSMX_Receiver_Serial2

} // namespace hf
