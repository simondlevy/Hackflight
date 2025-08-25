#include <Bitcraze_PMW3901.h>

Bitcraze_PMW3901 flow(PB4);

void setup() 
{
    Serial.begin(115200);

    if (!flow.begin()) {
        while (true) {
            Serial.println("Initialization of the flow sensor failed");
            delay(500);
        }
    }
}

int16_t deltaX,deltaY;

void loop() {
    // Get motion count since last call
    flow.readMotionCount(&deltaX, &deltaY);

    Serial.print("X: ");
    Serial.print(deltaX);
    Serial.print(", Y: ");
    Serial.print(deltaY);
    Serial.print("\n");

    delay(100);
}
