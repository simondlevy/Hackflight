/**
 *
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <motors.h>
#include <system.h>
#include <tasks/imu.hpp>
#include <tasks/zranger.hpp>

#include <Wire.h>

#include <VL53L1X.h>

static const uint8_t FLOWDECK_CS_PIN = PB4;

// Helpers -------------------------------------------------------------------

static void error(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

// ZRangerTask ---------------------------------------------------------------

static VL53L1X vl53l1;

void ZRangerTask::hardware_init()
{
    static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;

    if (!vl53l1.init()) {
        error("ZRANGER: Z-down sensor [FAIL]");
    }

    vl53l1.setDistanceMode(VL53L1X::Medium);

    vl53l1.setMeasurementTimingBudget(25000); // usec

    vl53l1.startContinuous(25); // msec
}

float ZRangerTask::hardware_read()
{
    return vl53l1.read();
}

// IMUTask -------------------------------------------------------------------

void ImuTask::deviceInit(void)
{
}

void ImuTask::readGyroRaw(Axis3i16* dataOut)
{
    (void)dataOut;
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    (void)dataOut;
}

// LED -----------------------------------------------------------------------

static const uint8_t LED_PIN = PC0;

const bool systemIsLedInverted() 
{
    return true;
}
 

// UART ----------------------------------------------------------------------

HardwareSerial Serial2(USART2);

bool systemUartReadByte(uint8_t * byte)
{
    const bool available = Serial2.available() > 0;

    if (available) {
        *byte = Serial2.read();
    }

    return available;
}

void systemUartWriteByte(const uint8_t byte)
{
    Serial2.write(byte);
}

// Motors ---------------------------------------------------------------------

int motorsGetRatio(uint32_t id)
{
    return 0;
}

void  motorsInit(void)
{
}

bool motorsTest(void)
{
    return true;
}

void  motorsSetRatios(const uint16_t ratios[])
{
    (void)ratios;
}

void  motorsStop(void)
{
}


// Main -----------------------------------------------------------------------

void setup() 
{
    Serial.begin(115200);

    Serial2.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    systemInit(LED_PIN, FLOWDECK_CS_PIN);
}

void loop() 
{
}
