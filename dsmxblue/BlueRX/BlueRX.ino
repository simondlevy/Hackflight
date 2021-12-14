/*
   ESP32 Bluetooth receiver for six-channel messages

   Copyright (c) Simon D. Levy 2021
 */


#include <BluetoothSerial.h>

#include "parser.hpp"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

class MyParser : public Parser {

    private:

    uint8_t _payload[128] = {};

        void handle_SET_NORMAL_RC(float  thr)
        {
            //static uint32_t count;
            //printf("%04d:   %+3.3f\n", count++, thr);
        }

    protected:

        virtual void collectPayload(uint8_t index, uint8_t value) override
        {
            _payload[index] = value;
        }

        virtual void dispatchMessage(uint8_t command) override
        {
            switch (command) {

                case 204:
                    {
                        float thr = 0;
                        memcpy(&thr,  &_payload[0], sizeof(float));

                        handle_SET_NORMAL_RC(thr);
                    } break;

            } // switch (_command)

        } // dispatchMessage 

}; // class MyParser


static BluetoothSerial SerialBT;
static MyParser parser;

void setup()
{
    parser.begin();
    Serial.begin(115200);
    SerialBT.begin("ESP32test3"); //Bluetooth device name
}

void loop()
{
    if (SerialBT.available()) {
        parser.parse(SerialBT.read());
    }
}
