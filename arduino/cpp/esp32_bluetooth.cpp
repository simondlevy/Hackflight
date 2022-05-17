/*
   ESP32 Bluetooth support

   Copyright (c) Simon D. Levy 2021
 */


#include <BluetoothSerial.h>

extern uint8_t bluetoothByte; 
extern bool bluetoothAvailable; 

static BluetoothSerial SerialBT;

void bluetoothStart(void)
{
    SerialBT.begin("ESP32test3"); //Bluetooth device name
}

bool bluetoothUpdate(void)
{
    bluetoothAvailable = SerialBT.available();
}

void bluetoothRead(void)
{
    bluetoothByte = SerialBT.read();
}
