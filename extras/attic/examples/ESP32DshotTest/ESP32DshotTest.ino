/*
   Test TinyPICO running DSHOT600 ESC protocol

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include "hackflight.hpp"
#include "motors/esp32dshot600.hpp"

static const uint8_t PINS[1] = {25};

hf::Esp32DShot600 motors = hf::Esp32DShot600(PINS, 1);

static uint8_t state;

void setup(void)
{
    Serial.begin(115200);
    motors.init();

    state = 0;
}

void loop(void)
{
    switch (state) {

        case 0:  
            Serial.println("Hit Enter to arm");
            if (Serial.available()) {
                Serial.read();
                motors.arm();
                state = 1;
            }
            delay(1000);
            break;

        case 1: 
            Serial.println("Hit Enter to start motor");
            if (Serial.available()) {
                Serial.read();
                motors.write(0, 0.1);
                state = 2;
            }
            delay(1000);
            break;

        case 2: 
            Serial.println("Hit Enter to stop motor");
            if (Serial.available()) {
                Serial.read();
                motors.write(0, 0);
                state = 3;
            }
            delay(1000);
            break;

        case 3: 
            Serial.println("Hit Enter disarm");
            if (Serial.available()) {
                Serial.read();
                motors.disarm();
                state = 0;
            }
            delay(1000);
            break;

        default:
            break;

    }
}
