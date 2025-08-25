#include <pmw3901.hpp>

#include <SPI.h>

static const uint8_t CS_PIN = 11;

static PMW3901 pmw3901;

static SPIClass spi = SPIClass(
        64, // MISO
        32, // SCK
        128); // MOSI

void setup() 
{
    Serial.begin(115200);

    spi.begin();

    if (!pmw3901.begin(CS_PIN, spi)) {

        while(true) { 
            Serial.println("Initialization of the flow pmw3901 failed");
            delay(500);
        }
    }
}

void loop() 
{
    /*
    int16_t deltaX = 0;
    int16_t deltaY = 0;
    bool gotMotion = false;

    pmw3901.readMotion(deltaX, deltaY, gotMotion); 

    Serial.print("deltaX: ");
    Serial.print(deltaX);
    Serial.print(",\tdeltaY: ");
    Serial.print(deltaY);
    Serial.print(",\tgotMotion: ");
    Serial.println(gotMotion ? "yes" : "no");*/

    Serial.print("MISO=");
    Serial.print(GPIO_PIN_6);

    Serial.print("  SCK=");
    Serial.print(GPIO_PIN_5);

    Serial.print("  MOSI=");
    Serial.println(GPIO_PIN_7);

    delay(100);
}
