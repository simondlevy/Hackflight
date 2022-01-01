/*
   TinyPICO-based transmitter

   Copyright (c) Simon D. Levy 2021

   MIT License
 */

#include <TinyPICO.h>

static const uint8_t THR_PIN = 25;
static const uint8_t ROL_PIN = 26;
static const uint8_t PIT_PIN = 27;
static const uint8_t YAW_PIN = 15;
static const uint8_t AU1_PIN = 14;
static const uint8_t AU2_PIN =  4;

static const uint8_t USB_PIN = 9;

static TinyPICO tp;

void setup(void)
{
    pinMode(AU1_PIN, INPUT_PULLUP);
    pinMode(AU2_PIN, INPUT_PULLUP);

    Serial.begin(115200);
}

void loop(void)
{
    static uint32_t batteryCount;

    // Smoothe-out fluctuations in battery detection
    batteryCount = tp.IsChargingBattery() ? batteryCount + 1 : 0;

    bool battery = batteryCount > 10;

    bool usb = digitalRead(USB_PIN);

    // Charging
    if (battery && usb) {
        tp.DotStar_SetPixelColor(255, 0, 0);
    }

    // Normal operation
    if (battery && !usb) {
        tp.DotStar_SetPixelColor(0, 255, 0);
    }

    if (!battery) {
        tp.DotStar_SetPixelColor(0, 0, 255);
    }

    /*
    if (battery) {
    }
    else {
        tp.DotStar_SetPixelColor(0, 255, 0);
    }

       printf("THR=%04d   ROL=%04d   PIT=%04d   YAW=%04d   AU1=%d   AU2=%d\n",
       analogRead(THR_PIN),
       analogRead(ROL_PIN),
       analogRead(PIT_PIN),
            analogRead(YAW_PIN),
            digitalRead(AU1_PIN),
            digitalRead(AU2_PIN)
          );*/

    delay(10);
}
