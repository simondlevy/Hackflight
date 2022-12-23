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

#include <Wire.h>
#include <SPI.h>

#include <VL53L5cx.h>
#include <PAA3905_MotionCapture.h>

#include <hackflight.h>
#include <debugger.h>

//#include <msp/parser.h>
//#include <espnow.h>
//#include <esp_now.h>

// VL53L5 -------------------------------------------------------------

static const uint8_t VL53L5_INT_PIN = 15; // Set to 0 for polling
static const uint8_t VL53L5_LPN_PIN = 2;

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

// PAA3905 -----------------------------------------------------------

static const uint8_t PAA3905_CS_PIN  = 5; 
static const uint8_t PAA3905_MOT_PIN = 32; 

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

// ------------------------------------------------------------------


//static const bool UART_INVERTED = false;
//static const uint8_t RX_PIN = 4; // unused
//static const uint8_t TX_PIN = 14;

//static MspSerializer _serializer;

void setup()
{
    Serial.begin(115200);

    pinMode(VL53L5_INT_PIN, INPUT);     

    Wire.begin();                
    Wire.setClock(400000);      
    delay(1000);

    if (VL53L5_INT_PIN > 0) {
        attachInterrupt(VL53L5_INT_PIN, rangerInterruptHandler, FALLING);
    }

    _ranger.begin();

    // Start SPI
    SPI.begin();

    delay(100);

    // Check device ID as a test of SPI communications
    if (!_mocap.begin()) {
        Debugger::reportForever("PAA3905 initialization failed");
    }

    Debugger::printf("Resolution is %0.1f CPI per meter height\n", _mocap.getResolution());

    pinMode(PAA3905_MOT_PIN, INPUT); 
    attachInterrupt(PAA3905_MOT_PIN, motionInterruptHandler, FALLING);

    // Start outgoing serial connection to FC, inverted
    //Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN, UART_INVERTED);
}

static void checkRanger(void)
{
    if (VL53L5_INT_PIN == 0 || _gotRangerInterrupt) {

        _gotRangerInterrupt = false;

        while (!_ranger.dataIsReady()) {
            delay(10);
        }

        _ranger.readData();

        for (auto i=0; i<_ranger.getPixelCount(); i++) {

            // Print per zone results 
            Debugger::printf("Zone : %2d, Nb targets : %2u, Ambient : %4lu Kcps/spads, ",
                    i, _ranger.getTargetDetectedCount(i), _ranger.getAmbientPerSpad(i));

            // Print per target results 
            if (_ranger.getTargetDetectedCount(i) > 0) {
                Debugger::printf("Target status : %3u, Distance : %4d mm\n",
                        _ranger.getTargetStatus(i), _ranger.getDistanceMm(i));
            }
            else {
                Debugger::printf("Target status : 255, Distance : No target\n");
            }
        }
        Debugger::printf("\n");
    } 

} // checkRanger

static void checkMocap(void)
{
    if (_gotMotionInterrupt) {

        _gotMotionInterrupt = false;

        _mocap.readBurstMode(); // use burst mode to read all of the data

        if (_mocap.motionDataAvailable()) { 

            static uint32_t _count;

            Debugger::printf("\n%05d ---------------------------------\n", _count++);

            if (_mocap.challengingSurfaceDetected()) {
                Debugger::printf("Challenging surface detected!\n");
            }

            int16_t deltaX = _mocap.getDeltaX();
            int16_t deltaY = _mocap.getDeltaY();

            uint8_t surfaceQuality = _mocap.getSurfaceQuality();
            uint8_t rawDataSum = _mocap.getRawDataSum();
            uint8_t rawDataMax = _mocap.getRawDataMax();
            uint8_t rawDataMin = _mocap.getRawDataMin();

            uint32_t shutter = _mocap.getShutter();

            PAA3905_MotionCapture::lightMode_t lightMode = _mocap.getLightMode();

            static const char * light_mode_names[4] = {"Bright", "Low", "Super-low", "Unknown"};
            Debugger::printf("%s light mode\n", light_mode_names[lightMode]);

            // Don't report X,Y if surface quality and shutter are under thresholds
            if (_mocap.dataAboveThresholds(lightMode, surfaceQuality, shutter)) {
                Debugger::printf("X: %d  Y: %d\n", deltaX, deltaY);
            }
            else {
                Debugger::printf("Data is below thresholds for X,Y reporting\n");
            }

            Debugger::printf("Number of Valid Features: %d, shutter: 0x%X\n",
                    4*surfaceQuality, shutter);
            Debugger::printf("Max raw data: %d  Min raw data: %d  Avg raw data: %d\n",
                    rawDataMax, rawDataMin, rawDataSum);
        }
    }

} // checkMocap

void loop()
{
    //static uint8_t _count;
    //Serial1.write(_count);
    //_count = (_count + 1) % 256;

    checkRanger();
    checkMocap();
}
