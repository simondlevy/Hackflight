#include <Adafruit_INA260.h>

static const uint8_t LED_PIN = A0; 

static Adafruit_INA260 _ina260;

static int _inc;

void setup() 
{
    pinMode(LED_PIN, OUTPUT);

    if (!_ina260.begin()) {
        while (true) {
            printf("Couldn't find INA260 chip\n");
            delay(500);
        }
    }

    _inc = +1;
}

void loop() 
{
    static int _level;

    analogWrite(LED_PIN, _level);

    static uint32_t _msec_prev;

    const auto msec_curr = millis();

    if (msec_curr - _msec_prev > 10) {
        _level += _inc;
        _inc = _level == 256 ? -1 : _level == 0 ? +1 : _inc;
        _msec_prev = msec_curr;

        printf("Current: %f mA | Bus Voltage: %f mV Power: %f mW\n",
                _ina260.readBusVoltage(),
                _ina260.readBusVoltage(),
                _ina260.readPower());
    }
}
