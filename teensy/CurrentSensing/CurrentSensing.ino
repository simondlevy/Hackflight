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
    static uint32_t _msec_prev;

    const auto msec_curr = millis();

    static int _level;

    if (msec_curr - _msec_prev > 10) {
        _level += _inc;
        _inc = _level == 256 ? -1 : _level == 0 ? +1 : _inc;
        _msec_prev = msec_curr;
    }

    printf("%d\n", _level);

    analogWrite(LED_PIN, _level);

    /*
    // Fade in
    for (int i = 0; i <= 255; i++) {
        analogWrite(LED_PIN, i);
        delay(10);
    }

    // Fade out
    for (int i = 255; i >= 0; i--) {
        analogWrite(LED_PIN, i);
        delay(10);
    }*/

    /*
    Serial.print("Current: ");
    Serial.print(_ina260.readCurrent());
    Serial.println(" mA");

    Serial.print("Bus Voltage: ");
    Serial.print(_ina260.readBusVoltage());
    Serial.println(" mV");

    Serial.print("Power: ");
    Serial.print(_ina260.readPower());
    Serial.println(" mW");
    Serial.println();
    */
}
