#include <Wire.h>
#include <VL53L1X.h>

static VL53L1X sensor;

void setup() 
{
    Serial.begin(115200);

    static TwoWire wire = TwoWire(PB7, PB6);

    wire.begin();
    wire.setClock(400000); // use 400 kHz I2C

    sensor.setBus(&wire);

    sensor.setTimeout(500);

    if (!sensor.init()) {
        while (true) {
            Serial.printf("Failed to detect and initialize sensor!\n");
            delay(500);
        }
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor.startContinuous(50);
}

void loop()
{
    Serial.print(sensor.read());

    if (sensor.timeoutOccurred()) {
        Serial.print(" TIMEOUT"); 
    }

    Serial.println();
}
