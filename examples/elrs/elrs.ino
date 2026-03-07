#include <CRSFforArduino.hpp>

static CRSFforArduino crsf;

static constexpr int CHANNEL_COUNT = 5;

static auto scalechan(serialReceiverLayer::rcChannels_t *rcChannels, const int k) -> float
{
    return map((float)crsf.readRcChannel(k), 989, 2012, -1, +1);
}

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {
        printf("t=%+3.3f\n", scalechan(rcChannels, 3));
    }
    delay(10);
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
