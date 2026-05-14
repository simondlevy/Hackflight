#include <Adafruit_INA260.h>

static const uint8_t LED_PIN = A0; 

static Adafruit_INA260 ina260;

void setup() 
{
    pinMode(LED_PIN, OUTPUT);

    if (!ina260.begin()) {
        while (true) {
            printf("Couldn't find INA260 chip\n");
            delay(500);
        }
    }
}

void loop() 
{
    // Fade in
    for (int i = 0; i <= 255; i++) {
        analogWrite(LED_PIN, i);
        delay(10);
    }

    // Fade out
    for (int i = 255; i >= 0; i--) {
        analogWrite(LED_PIN, i);
        delay(10);
    }
}
