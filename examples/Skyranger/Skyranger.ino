/*
   Copyright (c) 2023 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>

#include <VL53L5cx.h>
#include <PAA3905_MotionCapture.h>

#include "msp.h"

// Constants ----------------------------------------------------------

// Replace with the MAC Address of your ESPNOW receiver
static const uint8_t ESP_RECEIVER_ADDRESS[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x6C, 0x04};

// Pins
static const uint8_t VL53L5_INT_PIN  = 15;
static const uint8_t VL53L5_LPN_PIN  = 2;
static const uint8_t PAA3905_CS_PIN  = 5;
static const uint8_t PAA3905_MOT_PIN = 4;
static const uint8_t LED_PIN         = 25;

// Serial message IDs
static const uint8_t MSP_SET_ATTITUDE = 213;
static const uint8_t MSP_SET_VL53L5   = 221;
static const uint8_t MSP_SET_PAA3905  = 222;

// Helpers -----------------------------------------------------------

static void sendBytes(
        Msp & serializer,
        const uint8_t msgId,
        const int16_t data[],
        const uint8_t count)
{
    serializer.serializeShorts(msgId, data, count);

    // Send our sensor data to flight controller
    Serial.write(serializer.payload, serializer.payloadSize);

    // Send our sensor data to ESP-NOW receiver
    esp_now_send(ESP_RECEIVER_ADDRESS, serializer.payload, serializer.payloadSize);
}

static void updateLed(void)
{
    static uint32_t _prev;
    static bool _state;

    uint32_t msec = millis();

    if (msec - _prev > 500) {
        _state = !_state;
        digitalWrite(LED_PIN, _state);
        _prev = msec;
    }
}

static void reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void startEspNow(void)
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        reportForever("Error initializing ESP-NOW");
    }

    static esp_now_peer_info_t peerInfo;

    memcpy(peerInfo.peer_addr, ESP_RECEIVER_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        reportForever("Failed to add peer");
    }
}

// VL53L5 -------------------------------------------------------------

// Set to 0 for continuous mode
static const uint8_t VL53L5_INTEGRAL_TIME_MS = 10;

static VL53L5cx _ranger(
        Wire, 
        VL53L5_LPN_PIN, 
        VL53L5_INTEGRAL_TIME_MS,
        VL53L5cx::RES_4X4_HZ_1);

static volatile bool _gotRangerInterrupt;

static void rangerInterruptHandler() 
{
    _gotRangerInterrupt = true;
}

static void startRanger(void)
{
    Wire.begin();                
    Wire.setClock(400000);      
    delay(1000);

    pinMode(VL53L5_INT_PIN, INPUT);     

    if (VL53L5_INT_PIN > 0) {
        attachInterrupt(VL53L5_INT_PIN, rangerInterruptHandler, FALLING);
    }

    _ranger.begin();
}

static void checkRanger(Msp & serializer)
{
    static int16_t data[16];

    if (VL53L5_INT_PIN == 0 || _gotRangerInterrupt) {

        _gotRangerInterrupt = false;

        while (!_ranger.dataIsReady()) {
            delay(10);
        }

        _ranger.readData();

        for (auto i=0; i<_ranger.getPixelCount(); i++) {
            data[i] = _ranger.getDistanceMm(i);
        }
    } 

    sendBytes(serializer, MSP_SET_VL53L5, data, 16);
}

// PAA3905 -----------------------------------------------------------

PAA3905_MotionCapture _mocap(
        SPI,
        PAA3905_CS_PIN,
        PAA3905::DETECTION_STANDARD,
        PAA3905::AUTO_MODE_01,
        PAA3905::ORIENTATION_NORMAL,
        0x2A); // resolution 0x00 to 0xFF

static volatile bool _gotMotionInterrupt;

void motionInterruptHandler()
{
    _gotMotionInterrupt = true;
}

static void startMocap(void)
{
    // Start SPI
    SPI.begin();

    delay(100);

    // Check device ID as a test of SPI communications
    if (!_mocap.begin()) {
        reportForever("PAA3905 initialization failed");
    }

    pinMode(PAA3905_MOT_PIN, INPUT); 
    attachInterrupt(PAA3905_MOT_PIN, motionInterruptHandler, FALLING);
}

static void checkMocap(Msp & serializer)
{
    static int16_t data[2];

    if (_gotMotionInterrupt) {

        _gotMotionInterrupt = false;

        _mocap.readBurstMode(); // use burst mode to read all of the data

        if (_mocap.motionDataAvailable()) { 

            auto surfaceQuality = _mocap.getSurfaceQuality();

            auto shutter = _mocap.getShutter();

            PAA3905_MotionCapture::lightMode_t lightMode = _mocap.getLightMode();

            // Send X,Y if surface quality and shutter are above thresholds
            if (_mocap.dataAboveThresholds(lightMode, surfaceQuality, shutter)) {
                data[0] = _mocap.getDeltaX();
                data[1] = _mocap.getDeltaY();
            }
        }
    }

    sendBytes(serializer, MSP_SET_PAA3905, data, 2);
}


// Attitude messages from FC ----------------------------------------

void serialEvent(void)
{
    while (Serial.available()) {

        uint8_t byte = Serial.read();

        // Send message bytes directly to ESP receiver
        esp_now_send(ESP_RECEIVER_ADDRESS, &byte, 1);
    }
}

// ------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    startEspNow();

    startRanger();

    startMocap();
}

void loop()
{
    static Msp _serializer;

    checkRanger(_serializer);
    checkMocap(_serializer);

    updateLed();
}
