/*
   extras.cpp : Implementation of extra simulator functionality

   Copyright (C) Simon D. Levy, Matt Lubas, and Julio Hidalgo Lopez 2016

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

static const char * PORTNAME = "/dev/ttyUSB0";
static const int    BAUDRATE = 57600; // Fastest we could get from our XBees

#include "extras.hpp"
#include "serial.hpp"

#include "scriptFunctionData.h"
#include "v_repLib.h"

#include <stdio.h>
#include <algorithm>


static const char * sonarNames[4] = {"Sonar_Back", "Sonar_Front", "Sonar_Left", "Sonar_Right"};

static int sonarHandles[4];

static SerialConnection serialConnection(PORTNAME, BAUDRATE);

static bool serialConnected;

static int sonarDistances[4];

void extrasStart(void)
{
    for (int k=0; k<4; ++k)
        sonarHandles[k] = simGetObjectHandle(sonarNames[k]);

    serialConnected = serialConnection.openConnection();
}

void extrasUpdate(void)
{
    for (int k=0; k<4; ++k) {

        int sonarDistance = 0; 

        float detectedPoint[4];         // X,Y,Z,distance

        if (simReadProximitySensor(sonarHandles[k], detectedPoint, NULL, NULL) > 0)
            sonarDistance = (int)(detectedPoint[3] * 100);   // m to cm

        // Keep in MB142 range
        sonarDistances[k] = std::min(std::max(sonarDistance, 20), 765);

        //printf("%s: %d %c ", sonarNames[k], sonarDistances[k], k==3?'\n' : '|');
    }
}

void extrasMessage(int message, int * auxiliaryData, void * customData)
{
}


void extrasStop(void)
{
    if (serialConnected)
        serialConnection.closeConnection();
}

uint8_t Board::serialAvailableBytes(void)
{
    return serialConnected ? serialConnection.bytesAvailable() : 0;
}

uint8_t Board::serialReadByte(void)
{
    uint8_t c = 0;
    if (serialConnected)
        serialConnection.readBytes((char *)&c, 1);
    return c;
}

void Board::serialWriteByte(uint8_t c)
{
    if (serialConnected)
        serialConnection.writeBytes((char *)&c, 1);
}            

bool Board::sonarInit(uint8_t index) 
{
    return true;
}

void Board::sonarUpdate(uint8_t index)
{
}

uint16_t Board::sonarGetDistance(uint8_t index)
{
    return sonarDistances[index];
}
 
