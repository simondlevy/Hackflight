
#include <Arduino.h>

// https://github.com/simondlevy/SpektrumDSM
#include <SpektrumDSM.h>
static SpektrumDSM2048 rx;


void setup(void)
{  
    // Start receiver
    rx.begin();

    // Set up serial communication over USB
    Serial.begin(115200);
}


void loop(void)
{  
    static uint16_t values[5];

    values[0] = rx.getChannelValue(1); // roll
    values[1] = rx.getChannelValue(2); // pitch
    values[2] = rx.getChannelValue(3); // throttle
    values[3] = rx.getChannelValue(0); // yaw
    values[4] = rx.getChannelValue(5); // aux

    Serial.printf("%d %d %d %d %d\n", values[0], values[1], values[2], values[3], values[4]);
}


