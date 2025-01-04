/*
   ESP32 receiver sketch

   Does the following:

       - Relays SET_RC messages from transmitter to Teensy

       - Relays telemetry from Teensy to dongle

       - Sets LED color and blink rate based on status messages from Teensy

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// DotStar LED support
#include <TinyPICO.h>

// Hackflight support
#include <hackflight.hpp>
#include <espnow/utils.hpp>
#include <msp/parser.hpp>

// ESP-NOW comms -------------------------------------------------------------

static constexpr uint8_t TELEMETRY_DONGLE_ADDRESS[6] = {
    0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4
};

static constexpr uint8_t TRANSMITTER_ADDRESS[6] = {
    0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4
};

// Blinkenlights -------------------------------------------------------------

static const uint32_t LED_COLOR_FAILSAFE = 0xFF0000;
static const uint32_t LED_COLOR_HEARTBEAT = 0x00FF00;
static const uint32_t LED_COLOR_ARMED = 0xFF0000;
static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;
TinyPICO _tinypico;

// Handles EPS-NOW SET_RC messages from transmitter
void espnowEvent(const uint8_t * mac, const uint8_t * data, int len) 
{
    // Send SET_RC message to Teensy
    Serial1.write(data, len);
}

static uint8_t _status;

// Handles streaming messages from Teensy:  collects message bytes, and when a
// complete message is received, sends all the message bytes
// to Teensy.
void serialEvent1()
{
    static hf::MspParser _parser;

    static uint8_t _msg[256];
    static uint8_t _msgcount;

    while (Serial1.available()) {

        const auto c = Serial1.read();

        _msg[_msgcount++] = c;

        const auto msgid = _parser.parse(c);

        if (msgid) {

            // Handle status messag
            if (msgid == 122) {
                _status = _msg[5];
            }

            // Send telemetry to dongle
            else {
                hf::EspNowUtils::sendToPeer(TELEMETRY_DONGLE_ADDRESS,
                        _msg, _msgcount, "nano", "dongle");
            }

            _msgcount = 0;

        }
    }
}

void setup() 
{
    // Set up for serial debugging
    Serial.begin(115200);

    // Set up for receiving serial telemetry data from Teensy
    Serial1.begin(115200, SERIAL_8N1, 4, 14 );

    // Start ESP-NOW
    hf::EspNowUtils::init();

    // Add the telemetry dongle as a peer
    hf::EspNowUtils::addPeer(TELEMETRY_DONGLE_ADDRESS);

    // Add the transmitter as a peer
    hf::EspNowUtils::addPeer(TRANSMITTER_ADDRESS);

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(espnowEvent));
}

void loop() 
{
    if (_status == hf::STATUS_ARMED) {
        _tinypico.DotStar_SetPixelColor(LED_COLOR_ARMED);
    }

    else {

        const auto failsafe = _status == hf::STATUS_FAILSAFE;

        const auto blink_freq =
            failsafe ?  FAILSAFE_BLINK_RATE_HZ : HEARTBEAT_BLINK_RATE_HZ;

        const auto color = failsafe ? LED_COLOR_FAILSAFE : LED_COLOR_HEARTBEAT;

        static uint32_t _msec_prev;

        static uint32_t _delay_msec;

        const auto msec_curr = millis();

        if (msec_curr - _msec_prev > _delay_msec) {

            static bool _alternate;

            _msec_prev = msec_curr;

            _tinypico.DotStar_SetPixelColor(_alternate ? color : 0x000000);

            if (_alternate) {
                _alternate = false;
                _delay_msec = 100;
            }

            else {
                _alternate = true;
                _delay_msec = blink_freq * 1000;
            }
        }

    }
}
