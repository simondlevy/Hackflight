#include <Wire.h>   

#include "stm32f_reboot.hpp"

static const uint8_t LED_RED_L_PIN = PC0;
static const uint8_t LED_GREEN_L_PIN = PC1;
static const uint8_t LED_GREEN_R_PIN = PC2;
static const uint8_t LED_RED_R_PIN = PC3;

void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_RED_L_PIN, OUTPUT);

    Wire.begin();
}

void loop(void)
{
    while (Serial.available()) {

        if (Serial.read() == 'R') {
            reboot();
        }
    }
}
