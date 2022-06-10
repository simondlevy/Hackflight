#include <Arduino.h>

#include <led.h>

static const uint8_t PIN = 13;

static bool _on;

void ledInit(void)
{
    pinMode(PIN, OUTPUT);
}

void ledSet(bool on)
{
    digitalWrite(PIN, on);
    _on = on;
}

void ledToggle(void)
{
    _on = !_on;
    ledSet(_on);
}
