/*
   Copyright (c) 2022 Simon D. Levy

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
#include <board/stm32/stm32f4/stm32f411.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <debugger.h>
#include <esc/mock.h>
//#include <imu/real/softquat/bmi270.h>
#include <imu/mock.h>
#include <spihelper.h>
#include <receiver/real/sbus.h>

// IMU
static const uint8_t MOSI_PIN = PB_5;
static const uint8_t MISO_PIN = PB_4;
static const uint8_t SCLK_PIN = PB_3;
static const uint8_t CS_PIN   = PA_15;
// static const uint8_t EXTI_PIN = PA1;

#include <vector>
using namespace std;

//static const uint8_t LED_PIN  = PC13; // orange
static const uint8_t LED_PIN  = PC14; // blue

static SPIClass _spi(MOSI_PIN, MISO_PIN, SCLK_PIN);

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

static vector<PidController *> _pids = {&_anglePid};

static Stm32F411Board * _board;
//static Bmi270 * _imu;

static SbusReceiver _rx;

static Mixer _mixer = QuadXbfMixer::make();

void serialEvent2(void)
{
    _rx.read(Serial2);
}

static uint8_t _chipId1[4];
static uint8_t _chipId2[4];

static void readRegisters(uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    reg_addr = 0x80 | reg_addr;

    digitalWrite(CS_PIN, LOW);

    SPI.transfer(reg_addr);

    for (auto cnt = 0; cnt < length; cnt++) {
        *(reg_data + cnt) = SPI.transfer(0x00);
    }

    digitalWrite(CS_PIN, HIGH);
}

void setup(void)
{
    //static Bmi270 imu(RealImu::rotate270, _spi, CS_PIN);

    static MockImu imu;

    static MockEsc esc;

    static Stm32F411Board board(_rx, imu, _pids, _mixer, esc, LED_PIN);

    _board = &board;
    //_imu = &imu;

    Serial2.begin(100000, SERIAL_8E2);

    _board->begin();

    digitalWrite(CS_PIN, LOW);
    delay(1);
    digitalWrite(CS_PIN, HIGH);

    _spi.begin();
    //_spi.setBitOrder(MSBFIRST);
    //_spi.setClockDivider(calculateSpiDivisor(MAX_SPI_INIT_CLK_HZ));
    //_spi.setDataMode(SPI_MODE3);
    pinMode(CS_PIN, OUTPUT);

    delay(100);

    readRegisters(0x00, _chipId1, 4);
    delay(100);
    readRegisters(0x00, _chipId2, 4);
    delay(100);
}

static void dump(uint8_t data[4])
{
    Debugger::printf("%x %x %x %x", data[0], data[1], data[2], data[3]); 
}

void loop(void)
{
    _board->step();

    dump(_chipId1);
    Debugger::printf("    ");
    dump(_chipId2);
    Debugger::printf("\n");

    //Serial.println(_imu->id, HEX);
}
