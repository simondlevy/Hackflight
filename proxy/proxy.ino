/**
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

#include "msp/messages.h"
#include "msp/parser.hpp"
#include "msp/serializer.hpp"

static MspParser parser; 

static uint8_t status;

static bool armed;

void serialEvent1()
{
    while (Serial1.available()) {

        switch (parser.parse(Serial1.read())) {

            case MSP_SET_ARMING:
                armed = !armed;
                break;

            case MSP_SET_SETPOINT_RPYT:
                status = 1;
                break;

            case MSP_SET_SETPOINT_HOVER:
                status = 2;
                break;

        }
    }
}

void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    const char * msg[3] = {"idle", "rpyt", "hovering"};
    printf("%s: %s\n", armed ? "armed  " : "disarmed", msg[status]);
    delay(50);

    const float statevals[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

    static MspSerializer serializer;

    serializer.serializeFloats(MSP_STATE, statevals, 10);

    for (uint8_t k=0; k<serializer.payloadSize; ++k) {
        Serial1.write(serializer.payload[k]);
    }
}
