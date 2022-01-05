/*
   ESP32-based transmitter

   Copyright (c) Simon D. Levy 2021

   MIT License
 */

#include <TinyPICO.h>

//static const uint8_t THR_PIN = 25;
static const uint8_t ROL_PIN = A0;
//static const uint8_t PIT_PIN = 27;
//static const uint8_t YAW_PIN = 15;
static const uint8_t AU1_PIN = 13;
//static const uint8_t AU2_PIN =  4;

static const uint8_t USB_PIN = 9;

static TinyPICO tp;

typedef enum {

    POWER_USB,
    POWER_BATTERY,
    POWER_CHARGING

} power_state_t;

void setup(void)
{
    pinMode(AU1_PIN, INPUT_PULLUP);
    //pinMode(AU2_PIN, INPUT_PULLUP);

    //tp.DotStar_SetPixelColor(0, 255, 0);

    Serial.begin(115200);
}

static void fun(uint16_t val)
{
    int16_t MAXOUT = 100;
    int16_t MIDPOINT = 200;
    int16_t DEADBAND = 20;

    static const uint8_t LPF_SIZE = 5;

    static uint16_t lpfbuff[LPF_SIZE];
    static uint8_t lpfidx;

    lpfbuff[lpfidx] = val;
    lpfidx = (lpfidx + 1) % LPF_SIZE;

    uint32_t sum = 0;
    for (uint8_t k=0; k<LPF_SIZE; ++k) {
        sum += lpfbuff[k];
    }
    sum /= LPF_SIZE;

    printf("0 500 %d %d %d %d\n",
            sum, MIDPOINT, MIDPOINT-DEADBAND, MIDPOINT+DEADBAND);

    //return x < (MIDPOINT-DEADBAND) ? -100 : x > (MIDPOINT + DEADBAND) ? +100 : 0;
}

void loop(void)
{
/*
    static uint32_t batteryCount;
    static power_state_t powerStatePrev;

    // Smoothe-out fluctuations in battery detection
    batteryCount = tp.IsChargingBattery() ? batteryCount + 1 : 0;

    bool battery = batteryCount > 5;

    bool usb = digitalRead(USB_PIN);

    power_state_t powerState = battery ? (usb ? POWER_CHARGING : POWER_BATTERY) : POWER_USB;

    uint8_t red = powerState == POWER_CHARGING ? 255 : 0;
    uint8_t green = powerState == POWER_BATTERY ? 255 : 0;

    if (powerState != powerStatePrev) {

        tp.DotStar_SetPixelColor(red, green, 0);
    }

    powerStatePrev = powerState;

    printf("THR=%04d   ROL=%04d   PIT=%04d   YAW=%04d   AU1=%d   AU2=%d\n",
    analogRead(THR_PIN),
    analogRead(ROL_PIN),
    analogRead(PIT_PIN),
    analogRead(YAW_PIN),
    digitalRead(AU1_PIN),
    digitalRead(AU2_PIN)
    );

    printf("ROL=%04d   AU1=%d\n",
            analogRead(ROL_PIN),
            digitalRead(AU1_PIN));

*/
    fun(analogRead(ROL_PIN));

    //delay(5);
}
