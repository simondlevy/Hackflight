#include <Bitcraze_PMW3901.h>

Bitcraze_PMW3901 pmw3901(PB4);

void setup() 
{
    Serial.begin(115200);

    if (!pmw3901.begin()) {
        while (true) {
            Serial.println("Initialization of the flow sensor failed");
            delay(500);
        }
    }
}

void loop()
{
    
    int16_t deltaX=0, deltaY=0;

    pmw3901.readMotionCount(&deltaX, &deltaY);

    Serial.print("X: ");
    Serial.print(deltaX);
    Serial.print(", Y: ");
    Serial.print(deltaY);
    Serial.print("\n");

    delay(100);
}
