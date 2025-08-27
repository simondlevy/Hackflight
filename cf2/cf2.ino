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

// LED -----------------------------------------------------------------------

static const uint8_t LED_PIN = PC0;

const bool systemIsLedInverted() 
{
    return true;
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
