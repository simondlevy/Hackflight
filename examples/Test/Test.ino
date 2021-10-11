/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "HF_full.hpp"
#include "hf_mixers/quad/xmw.hpp"

#include "stream_serial.h"
#include "stream_motors.h"
#include "stream_receiver.h"
#include "stream_imu.h"
#include "stream_led.h"

static uint32_t LED_PIN = 13;

static constexpr float SCALE = 4.0f;
static constexpr float TRIM[3] = {0, 0.05, 0.035};
static hf::Receiver receiver = hf::Receiver(SCALE, TRIM);

static hf::MixerQuadXMW mixer;

static hf::HackflightFull h(&receiver, &mixer);

static bool running;

void setup(void)
{
    Serial1.begin(115200);

}

void loop(void)
{
    if (!running) {

        stream_startSerial();
        stream_startLed(LED_PIN);

        running = true;
    }

    static float motorvals[4]; // XXX needs to be static
    bool ledval = false;
    bool serialTaskReady = false;

    h.update(micros(), motorvals, &ledval, &serialTaskReady);

    if (serialTaskReady) {

        while (true) {
            stream_serialUpdate();
            if (stream_serialAvailable) {
                stream_serialRead();
                h.serialParse(stream_serialByte, motorvals);
            }
            else {
                break;
            }
        }

        while (h.serialAvailable() > 0) {
            stream_serialWrite(h.serialRead());
        }
    }

    stream_writeLed(LED_PIN, ledval);
}
