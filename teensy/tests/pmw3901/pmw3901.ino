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
    
    int16_t deltaX=0, deltaY=0;
    bool gotMotion = false;

    pmw3901.readMotion(deltaX, deltaY, gotMotion);

    printf("gotMotion=%s deltaX=%+03d deltaY=%+03d\n",
            gotMotion ? "yes" : "no ", deltaX, deltaY);

    delay(100);
}
