/*
   Spektrum DSMX support for Arduino flight controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "receiver.hpp"
#include <DSMRX.h>

namespace hf {

    class DSMX_ESP32_Serial1 : public Receiver {

        private:

            DSM2048 _rx;

            uint8_t _rxpin = 0;
            uint8_t _txpin = 0;  // unused

            static void receiverTask(void * params)
            {
                hf::DSMX_ESP32_Serial1 * receiver =  (hf::DSMX_ESP32_Serial1 *)params;

                while (true) {

                    if (Serial1.available()) {
                        receiver->handleSerialEvent(Serial1.read(), micros());
                    }

                    delay(1);
                }
            }

        protected:

            void begin(void)
            {
                // Start receiver on Serial1
                Serial1.begin(115000, SERIAL_8N1, _rxpin, _txpin);

                // Start the receiver timed task
                TaskHandle_t task;
                xTaskCreatePinnedToCore(hf::DSMX_ESP32_Serial1::receiverTask, "ReceiverTask", 10000, this, 1, &task, 1);
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

            DSMX_ESP32_Serial1(const uint8_t channelMap[6], const float demandScale, uint8_t rxpin, uint8_t txpin)
                :  Receiver(channelMap, demandScale) 
            { 
                _rxpin = rxpin;
                _txpin = txpin;
            }

            void handleSerialEvent(uint8_t value, uint32_t usec)
            {
                _rx.handleSerialEvent(value, usec);
            }

    }; // class DSMX_ESP32_Serial1

} // namespace hf
