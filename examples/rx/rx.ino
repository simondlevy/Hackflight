/*
   ESP32 dongle sketch

   Relays SET_RC messages from GCS program to onboard ESP32; receives telemetry
   from onboard ESP32 and relays to GCS.

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

static const uint32_t LED_FAILSAFE_COLOR = 0xFF0000;
static const uint32_t LED_HEARTBEAT_COLOR = 0x00FF00;
static const uint32_t LED_ARMED_COLOR = 0xFF0000;
static constexpr float HEARTBEAT_BLINK_RATE_HZ = 1.5;
static constexpr float FAILSAFE_BLINK_RATE_HZ = 0.25;
TinyPICO _tinypico;

// Failsafe ------------------------------------------------------------------
static uint32_t FAILSAFE_MSEC = 100;
static uint32_t _last_received_msec;

static void blinkLed(const bool gotFailsafe)
{
    const auto freq_hz =
        gotFailsafe ?
        FAILSAFE_BLINK_RATE_HZ :
        HEARTBEAT_BLINK_RATE_HZ;

    const auto color = gotFailsafe ? LED_FAILSAFE_COLOR : LED_HEARTBEAT_COLOR;

    static uint32_t _usec_prev;

    static uint32_t _delay_usec;

    const auto usec_curr = micros();

    if (usec_curr - _usec_prev > _delay_usec) {

        static bool _alternate;

        _usec_prev = usec_curr;

        _tinypico.DotStar_SetPixelColor(_alternate ? color : 0x000000);

        if (_alternate) {
            _alternate = false;
            _delay_usec = 100'100;
        }

        else {
            _alternate = true;
            _delay_usec = freq_hz * 1e6;
        }
    }
}


// Handles EPS-NOW SET_RC messages from transmitter, sending them to Teensy
// over UART.
void espnowEvent(const uint8_t * mac, const uint8_t * data, int len) 
{
    (void)mac;

    _last_received_msec = millis();

    Serial1.write(data, len);

}

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

        if (_parser.parse(c) == 121) { // STATE

            hf::EspNowUtils::sendToPeer(
                    TELEMETRY_DONGLE_ADDRESS, _msg, _msgcount, "nano", "dongle");

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
    static bool _failsafe;

    blinkLed(_failsafe);

    if (millis() - _last_received_msec > FAILSAFE_MSEC) {
        _failsafe = true;
    }
}
