/*
   Spektrum DSMX support for Arduino flight controllers

   MIT License
 */

#pragma once

#include "../HF_receiver.hpp"
#include <DSMRX.h>

namespace hf {

    class DSMX_Receiver : public Receiver {

        private:

            DSM2048 _rx;

        protected:

            virtual bool gotNewFrame(void) override
            {
                return _rx.gotNewFrame();
            }

            virtual void readRawvals(void) override
            {
                _rx.getChannelValues(rawvals, MAXCHAN);
            }

            virtual bool lostSignal(void) override
            {
                return _rx.timedOut(micros());
            }

        public:

            DSMX_Receiver(const uint8_t channelMap[6],
                          const float demandScale,
                          const float trim[3]=NULL)
                :  Receiver(channelMap, demandScale, trim) 
            { 
            }

            void handleSerialEvent(uint8_t value, uint32_t usec)
            {
                _rx.handleSerialEvent(value, usec);
            }

    }; // class DSMX_Receiver

} // namespace hf
