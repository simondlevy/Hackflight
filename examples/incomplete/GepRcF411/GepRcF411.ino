/*
   Copyright (c) 2023 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <SPI.h>

#include <hackflight.h>
#include <board/stm32/f/4/stm32f411.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <esc/mock.h>
#include <imu/mock.h>
#include <receiver/mock.h>

#include <vector>
using namespace std;

//static const uint8_t MOSI_PIN = PB15;
//static const uint8_t MISO_PIN = PB14;
//static const uint8_t SCLK_PIN = PB13;

static const uint8_t CS_PIN   = PA_4;

//static const uint8_t LED_PIN  = PC13; // orange
static const uint8_t LED_PIN  = PC14; // green

static Mixer mixer = QuadXbfMixer::make();

static MockEsc esc;

static MockReceiver rx;

static MockImu imu;

static vector<PidController *> pids = {};

static Stm32F411Board board(rx, imu, pids, mixer, esc, LED_PIN);

//SPIClass spi;

static void readRegisters(
        const uint8_t addr, uint8_t * buffer, const uint8_t count)
{
    digitalWrite(CS_PIN, LOW);
    buffer[0] = addr | 0x80;
    // spi.transfer(buffer, count+1);
    SPI.transfer(buffer, count+1);
    digitalWrite(CS_PIN, HIGH);
}

uint8_t readRegister(const uint8_t addr)
{
    uint8_t buffer[2] = {};
    readRegisters(addr, buffer, 1);
    return buffer[1];
}

static uint8_t chipId;

void setup(void)
{
    board.begin();

    //spi.setMOSI(MOSI_PIN);
    //spi.setMISO(MISO_PIN);
    //spi.setSCLK(SCLK_PIN);

    pinMode(CS_PIN, OUTPUT);

    digitalWrite(CS_PIN, LOW);
    delay(1);
    digitalWrite(CS_PIN, HIGH);
    delay(10);

    chipId = readRegister(0x00);
}

void loop(void)
{
    board.step();

    Board::printf("x%02X\n", chipId);
    //Board::printf("%x %x %x %x\n", PB_15, PB15, PC_13, PC13);
}
