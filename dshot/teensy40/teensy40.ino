#include "DSHOT.h"

void setup() 
{
    Serial.begin(115200);

    DSHOT_init(6);

}

void loop( ) 
{
    for (uint8_t k=0; k<25; ++ k) {
        uint16_t cmd[6] = {1000, 1000, 1000, 1000, 1000, 1000};
        DSHOT_send(cmd);
    }

    delayMicroseconds(225);
}
