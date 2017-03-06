/*
board.hpp : class header for board-specific routines

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

#pragma once

#include <cstdint>
#include "config.hpp"

namespace hf {

//abstraction for minimal board
class Board {
public: //interface
    //------------------------------------------- Core functionality ---------------------------------------------
    virtual void init() = 0;
    virtual const Config& getConfig() = 0;
    virtual void delayMilliseconds(uint32_t msec) = 0;
    virtual void dump(char * msg) = 0;
    virtual void imuRead(int16_t gyroAdc[3], int16_t accelAdc[3]) = 0;
    virtual uint64_t getMicros() = 0;
    virtual uint16_t readPWM(uint8_t chan)  = 0;
    virtual void writeMotor(uint8_t index, uint16_t value)  = 0;
    virtual void setLed(uint8_t id, bool is_on, float max_brightness = 255) { (void)id; (void)is_on; (void)max_brightness;}
    virtual void checkReboot(bool pendReboot)  {(void)pendReboot;}
    virtual void reboot(void) {}


    //-------------------------------------------------- RC -----------------------------------------------------
    virtual bool rcUseSerial(void) { return false; }
    virtual uint16_t rcReadSerial(uint8_t chan)  { (void)chan; return 0; }
    virtual bool rcSerialReady(void)  { return false; }


    //------------------------------------------------ Serial ---------------------------------------------------
    virtual uint8_t serialAvailableBytes(void) { return 0; };
    virtual uint8_t serialReadByte(void) { return 0; };
    virtual void serialWriteByte(uint8_t c) {(void)c;};


    //------------------------------------------------ extras ---------------------------------------------------
    virtual void extrasCheckSwitch(void)  {}
    virtual uint8_t  extrasGetTaskCount(void) { return 0; }
    virtual bool     extrasHandleMSP(uint8_t command) { (void)command; return true; }
    virtual void extrasPerformTask(uint8_t taskIndex) {(void)taskIndex;}
    //virtual void extrasInit(MSP * _msp) {} //TODO:// this causes circular includes

    //----------------------------------------------- Simulation -------------------------------------------------
    virtual void showArmedStatus(bool armed) {(void)armed;}
    virtual void showAuxStatus(uint8_t status) {(void)status;}

}; // class Board



} //namespace
