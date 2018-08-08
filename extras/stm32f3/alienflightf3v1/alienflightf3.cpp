#include <f3board.h>
#include <SpektrumDSM.h>

SpektrumDSM2048 * rx;

static uint8_t avail;

uint8_t dsmSerialAvailable(void)
{
    return avail;
}

uint8_t dsmSerialRead(void)
{
    avail--;

    return Serial2.read();
}

void serialEvent2()
{
    avail = 1;

    rx->handleSerialEvent(micros());
}

void setup() {
  
  Serial.begin(115200);

  Serial2.begin(115200);

  rx = new SpektrumDSM2048();
}

void loop() {

    if (rx->gotNewFrame()) {

        uint16_t values[8];

        rx->getChannelValues(values);

        for (int k=0; k<8; ++k) {
            Serial.printf("%d ", values[k]);
        }
        Serial.printf("\n");
    }

    // Allow some time between readings
    delay(10);  
}
