#include <SPI.h>

#include <pmw3901.hpp>

static const uint8_t CS_PIN = 3;

static PMW3901 pmw3901;

void setup() 
{
    Serial.begin(115200);

    SPI.begin();

    if (!pmw3901.begin(CS_PIN)) {
        while (true) {
            Serial.println("Initialization of the flow sensor failed");
            delay(500);
        }
    }
}

void loop()
{
    
    int16_t dx=0, dy=0;
    bool gotMotion = false;

    pmw3901.readMotion(dx, dy, gotMotion);

    printf("gotMotion=%s dx=%+03d dy=%+03d\n",
            gotMotion ? "yes" : "no ", dx, dy);

    delay(100);
}
