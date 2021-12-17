/*
   ESP32 Bluetooth support

   Copyright (c) Simon D. Levy 2021
 */


#include <BluetoothSerial.h>

extern uint8_t stream_bluetoothByte; 
extern bool stream_bluetoothAvailable; 

static BluetoothSerial SerialBT;

void stream_startBluetooth(void)
{
    SerialBT.begin("ESP32test3"); //Bluetooth device name
}

bool stream_bluetoothUpdate(void)
{
    stream_bluetoothAvailable = SerialBT.available();
}

void stream_bluetoothRead(void)
{
    stream_bluetoothByte = SerialBT.read();
}
