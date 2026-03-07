#include <CRSFforArduino.hpp>

static CRSFforArduino crsf;

static constexpr int CHANNEL_COUNT = 5;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe)
    {

        unsigned long thisTime = millis();
        static unsigned long lastTime = millis();

        if (thisTime < lastTime) {
            lastTime = thisTime;
        }

        if (thisTime - lastTime >= 100) {
            lastTime = thisTime;
            for (int i = 1; i <= CHANNEL_COUNT; i++) {
                // printf("c%d:%d ", i, crsf.rcToUs(crsf.getChannel(i)));
                printf("c%d:%d ", i, crsf.readRcChannel(i));
            }
            printf("\n");
        }
    }
}

void setup()
{
    Serial.begin(0);

    if (!crsf.begin()) {

        crsf.end();

        Serial.println("CRSF for Arduino initialisation failed!");

        while (1) {
            delay(10);
        }
    }

    crsf.setRcChannelsCallback(onReceiveRcChannels);
}

void loop()
{
    crsf.update();
}
