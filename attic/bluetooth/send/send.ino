/*
Communicate with computer via Bluetooth

Sends letters of the alphabet cyclically

Copyright (C) Simon D. Levy 2021

MIT License
*/

static uint8_t b;

static HardwareSerial * port = &Serial3;

void setup() 
{
    port->begin(115200);

    b = 0;
}

void loop() 
{
    char c = ('A' + b);

    port->write(c);

    b = (b + 1) % 26;

    delay(100);
}
