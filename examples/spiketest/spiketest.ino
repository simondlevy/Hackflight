#include <vector>

#include <dsmrx.hpp>

#include "difference_risp_train.hpp"

static EncoderHelper encoder_helper;
static DecoderHelper decoder_helper;

static Dsm2048 rx;

static void dump_decoder()
{
    float acts[decoder_helper.nout] = {};
    decoder_helper.get_actions(acts);
    for (size_t k=0; k<decoder_helper.nout; ++k) {
        printf("%+3.3f ", acts[k]);
    }
    printf("\n");
}

// Handles incoming spikes from Raspberry Pi
void serialEvent4()
{
    static bool got_spike;
    static uint8_t spike_count;

    while (Serial4.available()) {

        const uint8_t val = Serial4.read();

        if (got_spike) {
            decoder_helper.times[spike_count] = val;
            spike_count++;
            if (spike_count == decoder_helper.nout) {
                dump_decoder();
                spike_count = 0;
            }
        }
        else {
            decoder_helper.counts[spike_count] = val;
        }
        got_spike = !got_spike;
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

    for (size_t k=0; k<encoder_helper.nspikes; ++k) {
        const auto spike = spikes[k];
        const uint8_t id = spike.id;
        const uint8_t time = spike.time;
        Serial4.write(id);
        Serial4.write(time);
    }

    Serial4.write(0xFF); // sentinel byte
}
