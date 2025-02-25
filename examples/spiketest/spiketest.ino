#include <vector>

#include <dsmrx.hpp>

#include "difference_risp_train.hpp"

#include <hackflight.hpp>
#include <msp/parser.hpp>
#include <msp/serializer.hpp>

static EncoderHelper encoder_helper;
static DecoderHelper decoder_helper;

static Dsm2048 rx;

// Handles incoming spikes from Raspberry Pi
void serialEvent4()
{
    while (Serial4.available()) {

        static hf::MspParser parser;

        if (parser.parse(Serial4.read()) == 121) {

            decoder_helper.counts[0] = parser.getByte(1);
            decoder_helper.times[0] = parser.getByte(2);
            float acts[decoder_helper.nout] = {};
            decoder_helper.get_actions(acts);
            printf("%f\n", acts[0]);
        }
    }
}

// Handles incoming messages from DSMX receiver
void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.parse(Serial1.read(), micros());
    }
}

void setup()
{
    encoder_helper.init();

    decoder_helper.init();

    // Start debugging
    Serial.begin(115200);

    // Start communications with DSMX receiver
    Serial1.begin(115200);

    // Start communications with Raspberry Pi
    Serial4.begin(115200);
}

void loop()
{
    static float chanvals[6];

    if (rx.gotNewFrame()) {

        rx.getChannelValuesMlp6Dsm(chanvals);
    }

    float obs[2] = {-chanvals[1], chanvals[2]};

    encoder_helper.get_spikes(obs);

    const auto spikes = encoder_helper.spikes;

    uint8_t msg[256];
    msg[0] = encoder_helper.nspikes;

    for (size_t k=0; k<encoder_helper.nspikes; ++k) {
        const auto spike = spikes[k];
        msg[2*k+1] = spike.id;
        msg[2*k+2] = spike.time;
    }

    static hf::MspSerializer serializer;

    serializer.serializeBytes(121, msg, 2 * encoder_helper.nspikes + 1);

    Serial4.write(serializer.payload, serializer.payloadSize);
}
