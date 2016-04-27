#include <ReceiverPoll.h>
#include <Arduino.h>

static int s_channel1_pin;
static int s_channel2_pin;
static int s_channel3_pin;
static int s_channel4_pin;
static int s_channel5_pin;

static void set_pin(int pin, int & channel_pin) {

    channel_pin = pin;
    pinMode(pin, INPUT);
}

void rxGetChannelValues(short * channels) {

    channels[0]  = pulseIn(s_channel1_pin, HIGH);
    channels[1]  = pulseIn(s_channel2_pin, HIGH);
    channels[2]  = pulseIn(s_channel3_pin, HIGH);
    channels[3]  = pulseIn(s_channel4_pin, HIGH);
    channels[4]  = pulseIn(s_channel5_pin, HIGH);
}

void rxInitChannels(int channel1_pin, int channel2_pin, int channel3_pin, int channel4_pin, int channel5_pin) {

    set_pin(channel1_pin, s_channel1_pin);
    set_pin(channel2_pin, s_channel2_pin);
    set_pin(channel3_pin, s_channel3_pin);
    set_pin(channel4_pin, s_channel4_pin);
    set_pin(channel5_pin, s_channel5_pin);
}
