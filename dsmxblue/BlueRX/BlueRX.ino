/*
   ESP32 Bluetooth receiver for six-channel messages

   Copyright (c) Simon D. Levy 2021
 */


#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

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
