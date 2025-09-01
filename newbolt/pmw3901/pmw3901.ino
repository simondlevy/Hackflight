#include <SPI.h>

#include <pmw3901.hpp>

static const uint8_t CS_PIN = PB4;

static PMW3901 pmw3901;

static SPIClass spi;

void setup() 
{
    Serial.begin(115200);

    spi.setSCLK(PA5);
    spi.setMISO(PA6);
    spi.setMOSI(PA7);

    spi.begin();

    if (!pmw3901.begin(CS_PIN, spi)) {
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

    Serial.print("gotMotion = ");
    Serial.print(gotMotion ? "yes" : "no");
    Serial.print("  deltaX=");
    Serial.print(deltaX);
    Serial.print("  deltaY=");
    Serial.println(deltaY);

    delay(100);
}
