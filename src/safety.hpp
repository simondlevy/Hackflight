/*
   Arming protocol

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

// Failsafe ------------------------------------------------------------------
static uint32_t FAILSAFE_MSEC = 100;
static uint32_t _last_received_msec;

// Arming  -------------------------------------------------------------------
static bool _was_arming_switch_on;
static bool _is_armed;


// Handles EPS-NOW SET_RC messages from transmitter
void espnowEvent(const uint8_t * mac, const uint8_t * data, int len) 
{
    (void)mac;

    _last_received_msec = millis();

    static hf::MspParser _parser;

    for (int k=0; k<len; ++k) {
        
        if (_parser.parse(data[k])) {

            const auto c1 = _parser.getUshort(0);
            const auto c5 = _parser.getUshort(4);

            const auto is_arming_switch_on = c5 > 1500;

            // Arm vehicle if safe
            if (is_arming_switch_on) {
                if (!_was_arming_switch_on && c1 < 1050) {
                    _is_armed = true;
                }
            }

            // Disarm
            else {
                _is_armed = false;
            }

            _was_arming_switch_on = is_arming_switch_on;
        }
    }

    // Send message along to Teensy
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

        if (_parser.parse(c)) {

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

    // Show steady red LED if armed
    if (_is_armed) {
        _tinypico.DotStar_SetPixelColor(LED_ARMED_COLOR);
    }

    // Otherise, blink LED at appropriate color and rate
    else {

        const auto freq_hz =
            _failsafe ?
            FAILSAFE_BLINK_RATE_HZ :
            HEARTBEAT_BLINK_RATE_HZ;

        const auto color = _failsafe ? LED_FAILSAFE_COLOR : LED_HEARTBEAT_COLOR;

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

    // Check failsafe
    if (millis() - _last_received_msec > FAILSAFE_MSEC) {
        _failsafe = true;
    }
}
