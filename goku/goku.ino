#include "bootloader.hpp"

static const uint8_t LED_PIN = PC14;

static void blinkLed()
{
    digitalWrite(LED_PIN, HIGH);  
    delay(1000);            
    digitalWrite(LED_PIN, LOW); 
    delay(1000);   
}

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
}

void loop()
{  
    blinkLed();

    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}
