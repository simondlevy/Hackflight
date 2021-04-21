/*
   DSMX to SBUS actuator

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <SBUS.h>

namespace hf {

    class Dsmx2Sbus : public rft::Actuator {

        protected:

            void begin(void)
            {
            }

            void cut(void)
            {
            }

            virtual void runDisarmed(void) override
            {
                Serial.println("disarmed");
            }

        public:

            void run(float * demands)
            {
                Serial.println("armed");
            }

    }; // class Dsmx2Sbus

} // namespace hf

/*
DSM2048 rx;

SBUS sbus = SBUS(Serial2);

void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

void setup(void)
{
    Serial1.begin(115000);

    sbus.begin();
}

void loop(void)
{
    static float outvals[16] = {};

    if (rx.timedOut(micros())) {
        Serial.println("*** TIMED OUT ***");
    }

    else if (rx.gotNewFrame()) {

        float invals[8] = {};

        rx.getChannelValuesNormalized(invals, 8);

        outvals[0] = invals[0];
        outvals[1] = invals[1];
        outvals[2] = invals[2];
        outvals[3] = invals[3];
        outvals[4] = invals[6];
        outvals[5] = invals[4];
    }

    sbus.writeCal(outvals);
}
*/
