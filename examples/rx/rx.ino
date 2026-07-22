/* Hackflight ESP32 transmitter sketch
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <esp_now.h>
#include <WiFi.h>

#include <hackflight.h>
#include <firmware/debugger.hpp>

static const uint8_t kTransmitterAddress[6] = {0xD4,0xD4,0xDA,0xAA,0x2E,0xF0};
static const uint8_t kDongleAddress[6] = {0xD4,0xD4,0xDA,0x83,0x97,0x90};

static void SetupWifi()
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        ReportForever("Error initializing ESP-NOW");
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kReceiverAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        ReportForever("Failed to add peer");
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(KLedPin, OUTPUT);
    digitalWrite(KLedPin, HIGH);

    pinMode(kArmingPin, INPUT);

    arming_prev_ = digitalRead(kArmingPin);
}

void loop()
{
    static bool armed_;
    const auto arming_curr = digitalRead(kArmingPin);
    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }
    arming_prev_ = arming_curr;

    const auto hovering = hoverButton_.Read();

    const auto autopilot = autopilotButton_.Read();

    const auto throttle = 1 - ReadGimbal(kThrottlePin);

    const auto roll = 2 * (0.5 - ReadGimbal(kRollPin));

    const auto pitch = 2 * (ReadGimbal(kPitchPin) - 0.5);

    const auto yaw = 2 * ReadGimbal(kYawPin) - 1;

    const auto volts = voltage_divider_.read();

    if (volts < kLowVoltage) {
        blinkLeds();
    }

    Serial.printf("throttle=%3.2f roll=%+3.2f pitch=%+3.2f yaw=%+3.2f | "
            "armed=%d hovering=%d autopilot=%d | voltage=%3.3f\n",
            throttle, roll, pitch, yaw, armed_, hovering, autopilot, volts);
}
