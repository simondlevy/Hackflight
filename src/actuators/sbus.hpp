/*
   Actuator with SBUS output

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <SBUS.h>

namespace hf {

    class SbusActuator : public rft::Actuator {


        private:

            SBUS sbus = SBUS(Serial2);

            float outvals[16] = {};

            void send(void)
            {
                sbus.writeCal(outvals);
            }

        protected:

            void begin(void)
            {
                sbus.begin();
            }

            void cut(void)
            {
            }

            virtual void runDisarmed(void) override
            {
                outvals[0] = -1;
                outvals[1] = 0;
                outvals[2] = 0;
                outvals[3] = 0;
                outvals[4] = -1;
                outvals[5] = -1;

                send();
            }

        public:

            void run(float * demands)
            {
                outvals[0] = demands[0];
                outvals[1] = demands[1];  
                outvals[2] = -demands[2];// XXX Why must this be reversed?
                outvals[3] = demands[3];
                outvals[4] = +1;
                outvals[5] = -1;

                send();
            }

    }; // class SbusActuator

} // namespace hf
