/*
   Copyright (c) 2022 Simon D. Levy

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


//  Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

#include <PAA3905_MotionCapture.h>

#include <hackflight.h>
#include <debugger.h>
#include "msp/arduino.h"

// MCU choice --------------------------------------------------------

static const uint8_t SR_CS_PIN  = 5;
static const uint8_t SR_MOT_PIN = 4;

static const uint8_t TP_CS_PIN  = 5;
static const uint8_t TP_MOT_PIN = 32;

// TP
static const uint8_t RX1_PIN = 15;
static const uint8_t TX1_PIN = 27;

// MSP message IDs ----------------------------------------------------

static const uint8_t MOCAP_MSG_TYPE  = 122;  // PAA3905 motion capture

// Helper -------------------------------------------------------------

static void sendData(
        ArduinoMsp & msp,
        const uint8_t messageType,
        const int16_t data[],
        const uint8_t count) 
{
    msp.serializeShorts(messageType, data, count);
    msp.sendPayload(Serial1);
}

// PAA3905 -----------------------------------------------------------

static const uint8_t PAA3905_CS_PIN  = TP_CS_PIN; 
static const uint8_t PAA3905_MOT_PIN = TP_MOT_PIN; 

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
        HfDebugger::reportForever("PAA3905 initialization failed");
    }

    HfDebugger::printf(
            "Resolution is %0.1f CPI per meter height\n", _mocap.getResolution());

    pinMode(PAA3905_MOT_PIN, INPUT); 
    attachInterrupt(PAA3905_MOT_PIN, motionInterruptHandler, FALLING);
}

static void checkMocap(ArduinoMsp & msp)
{
    static int16_t data[2];

    if (_gotMotionInterrupt) {

        static uint32_t count;

        HfDebugger::printf("mocap %d\n", count++);

        _gotMotionInterrupt = false;

        _mocap.readBurstMode(); // use burst mode to read all of the data

        if (_mocap.motionDataAvailable()) { 

            uint8_t surfaceQuality = _mocap.getSurfaceQuality();

            uint32_t shutter = _mocap.getShutter();

            PAA3905_MotionCapture::lightMode_t lightMode = _mocap.getLightMode();

            // Send X,Y if surface quality and shutter are above thresholds
            if (_mocap.dataAboveThresholds(lightMode, surfaceQuality, shutter)) {
                data[0] = _mocap.getDeltaX();
                data[1] = _mocap.getDeltaY();
            }
        }
    }

    sendData(msp, MOCAP_MSG_TYPE, data, 2);
}

// ------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

    startMocap();
}

void loop()
{
    static ArduinoMsp _msp;

    checkMocap(_msp);
}
