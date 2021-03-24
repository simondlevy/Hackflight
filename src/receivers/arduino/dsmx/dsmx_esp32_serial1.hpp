/*
   Spektrum DSMX support for ESP32 flight controllers using Serial1

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "receivers/arduino/dsmx.hpp"
#include <DSMRX.h>

static hf::DSMX_Receiver * _dsmx_rx;

void serialEvent1(void)
{
    while (Serial1.available()) {

        _dsmx_rx->handleSerialEvent(Serial1.read(), micros());
    }
}

namespace hf {

    class DSMX_Receiver_ESP32_Serial1 : public DSMX_Receiver {

        private:

            uint8_t _rxpin = 0;
            uint8_t _txpin = 0; // unused

            static void timertask(void * params)
            {

            DSMX_Receiver_ESP32_Serial1 * receiver = (DSMX_Receiver_ESP32_Serial1 *)params; 

              while (true) {

                    if (Serial1.available()) {
                        receiver->handleSerialEvent(Serial1.read(), micros());
                    }

                    delay(1);
                }
            }

        protected:

            void begin(void) override 
            {
                Receiver::begin();

                Serial1.begin(115000, SERIAL_8N1, _rxpin, _txpin);

                TaskHandle_t task;
                xTaskCreatePinnedToCore(timertask, "TimerTask", 10000, this, 1, &task, 1);
            }

        public:

            DSMX_Receiver_ESP32_Serial1(const uint8_t channelMap[6], const float demandScale, uint8_t rxpin, uint8_t txpin)
                :  DSMX_Receiver(channelMap, demandScale) 
            { 
                _dsmx_rx = this;

                _rxpin = rxpin;
                _txpin = txpin;
            }

    }; // class DSMX_Receiver_ESP32_Serial1

} // namespace hf
