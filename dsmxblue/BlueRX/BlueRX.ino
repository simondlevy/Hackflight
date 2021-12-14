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

        void handle_SET_NORMAL_RC(float  thr, float  rol, float  pit, float  yaw, float  aux1, float  aux2)
        {
            // XXX
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

                        float rol = 0;
                        memcpy(&rol,  &_payload[4], sizeof(float));

                        float pit = 0;
                        memcpy(&pit,  &_payload[8], sizeof(float));

                        float yaw = 0;
                        memcpy(&yaw,  &_payload[12], sizeof(float));

                        float aux1 = 0;
                        memcpy(&aux1,  &_payload[16], sizeof(float));

                        float aux2 = 0;
                        memcpy(&aux2,  &_payload[20], sizeof(float));

                        handle_SET_NORMAL_RC(thr, rol, pit, yaw, aux1, aux2);
                    } break;

            } // switch (_command)

        } // dispatchMessage 

}; // class MyParser


static BluetoothSerial SerialBT;

void setup()
{
    Serial.begin(115200);
    SerialBT.begin("ESP32test3"); //Bluetooth device name
}

void loop()
{
    if (SerialBT.available()) {
        Serial.println(SerialBT.read(), HEX);
    }
}
