/*
HardwareWire.cpp : Wire API impelentation for BreezySTM32 library

Copyright (C) 2017 Simon D. Levy 

This file is part of BreezySTM32.

BreezySTM32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BreezySTM32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
*/

extern "C" {

#include <Arduino.h>
#include <Wire.h>

typedef enum I2CDevice {
    I2CDEV_1,
    I2CDEV_2,
    I2CDEV_MAX = I2CDEV_2
} I2CDevice;

typedef enum {
    READ,
    WRITE
} i2cJobType_t;

enum {
    I2C_JOB_DEFAULT,
    I2C_JOB_QUEUED,
    I2C_JOB_BUSY,
    I2C_JOB_COMPLETE,
    I2C_JOB_ERROR
};

typedef struct i2cJob{
    i2cJobType_t type;
    uint8_t addr;
    uint8_t reg;
    uint8_t* data;
    uint8_t length;
    struct i2cJob* next_job;
    volatile uint8_t* _status;
    void (*CB)(void);
} i2cJob_t;

#define I2C_BUFFER_SIZE 64

#define I2C_DEVICE (I2CDEV_2)

#include <stdbool.h>
#include <stdio.h>
#include <string.h> // memset

#include "stm32f10x_conf.h"
#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"

// I2C2
// SCL  PB10
// SDA  PB11
// I2C1
// SCL  PB6
// SDA  PB7

// I2C Interrupt Handlers
static void i2c_ev_handler(void);

// I2C Circular Buffer Variables
static i2cJob_t i2c_buffer[I2C_BUFFER_SIZE];
static volatile uint8_t i2c_buffer_head;
static volatile uint8_t i2c_buffer_tail;
static volatile uint8_t i2c_buffer_count;

typedef struct i2cDevice_t {
    I2C_TypeDef *dev;
    GPIO_TypeDef *gpio;
    uint16_t scl;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t peripheral;
} i2cDevice_t;

static const i2cDevice_t i2cHardwareMap[] = {
    { I2C1, GPIOB, Pin_6, Pin_7, I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_APB1Periph_I2C1 },
    { I2C2, GPIOB, Pin_10, Pin_11, I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_APB1Periph_I2C2 },
};

// Copy of peripheral address for IRQ routines
static I2C_TypeDef *I2Cx = NULL;

// Copy of device index for reinit, etc purposes
static I2CDevice I2Cx_index;

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler();
}

#define I2C_DEFAULT_TIMEOUT 30000

static volatile bool      _error;
static volatile bool      _busy;
static volatile uint8_t   _addr;
static volatile uint8_t   _reg;
static volatile uint8_t   _data;
static volatile uint8_t   _bytes;
static volatile uint8_t   _writing;
static volatile uint8_t   _reading;
static volatile uint8_t * _write_p;
static volatile uint8_t * _read_p;
static volatile uint8_t * _status;

static void i2cUnstick(void)
{
    GPIO_TypeDef *gpio;
    gpio_config_t cfg;
    uint16_t scl, sda;
    int i;

    // prepare pins
    gpio = i2cHardwareMap[I2Cx_index].gpio;
    scl = i2cHardwareMap[I2Cx_index].scl;
    sda = i2cHardwareMap[I2Cx_index].sda;

    digitalHi(gpio, scl | sda);

    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_Out_OD;
    gpioInit(gpio, &cfg);

    for (i = 0; i < 8; i++) {
        // Wait for any clock stretching to finish
        while (!digitalIn(gpio, scl))
            delayMicroseconds(10);

        // Pull low
        digitalLo(gpio, scl); // Set bus low
        delayMicroseconds(10);
        // Release high again
        digitalHi(gpio, scl); // Set bus high
        delayMicroseconds(10);
    }

    // Generate a start then stop condition
    // SCL  PB10
    // SDA  PB11
    digitalLo(gpio, sda); // Set bus data low
    delayMicroseconds(10);
    digitalLo(gpio, scl); // Set bus scl low
    delayMicroseconds(10);
    digitalHi(gpio, scl); // Set bus scl high
    delayMicroseconds(10);
    digitalHi(gpio, sda); // Set bus sda high

    // Init pins
    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_AF_OD;
    gpioInit(gpio, &cfg);
}


void i2c_ev_handler(void)
{
    static uint8_t subaddress_sent, final_stop;                         // flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                // index is signed -1 == send the subaddress
    uint8_t SReg_1 = I2Cx->SR1;                                         // read the _status register here

    if (SReg_1 & 0x0001) {                                              // we just sent a start - EV5 in ref manual
        I2Cx->CR1 &= ~0x0800;                                           // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                            // make sure ACK is on
        index = 0;                                                      // reset the index
        if (_reading && (subaddress_sent || 0xFF == _reg)) {            // we have sent the subaddr
            subaddress_sent = 1;                                        // make sure this is set in case of no subaddress, so following code runs correctly
            if (_bytes == 2)
                I2Cx->CR1 |= 0x0800;                                    // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, _addr, I2C_Direction_Receiver);    // send the address and set hardware mode
        } else {                                                        // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, _addr, I2C_Direction_Transmitter); // send the address and set hardware mode
            if (_reg != 0xFF)                                            // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                             // send a subaddress
        }

    } else if (SReg_1 & 0x0002) {                                       // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                        // memory fence to control hardware
        if (_bytes == 1 && _reading && subaddress_sent) {                 // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                       // turn off ACK
            __DMB();
            (void)I2Cx->SR2;                                            // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                             // program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        } else {                                                        // EV6 and EV6_1
            (void)I2Cx->SR2;                                            // clear the ADDR here
            __DMB();
            if (_bytes == 2 && _reading && subaddress_sent) {             // rx 2 _bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to fill
            } else if (_bytes == 3 && _reading && subaddress_sent)        // rx 3 _bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // make sure RXNE disabled so we get a BTF in two _bytes time
            else                                                        // receiving greater than three _bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }

    } else if (SReg_1 & 0x004) {                                        // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1;
        if (_reading && subaddress_sent) {                               // EV7_2, EV7_3
            if (_bytes > 2) {                                            // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                _read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // program the Stop
                final_stop = 1;                                         // required to fix hardware
                _read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            } else {                                                    // EV7_3
                if (final_stop){
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                }
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                _read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                _read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N
                index++;                                                // to show job completed
            }
        } else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (subaddress_sent || (_writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                index++;                                                // to show that the job is complete
            } else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);                        // program the repeated Start
                subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CR1 & 0x0100) {
            ;
        }

    } else if (SReg_1 & 0x0040) {                                       // Byte received - EV7
        _read_p[index++] = (uint8_t)I2Cx->DR;
        if (_bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (_bytes == index)                                             // We have completed a final EV7
            index++;                                                    // to show job is complete
    } else if (SReg_1 & 0x0080) {                                       // Byte transmitted EV8 / EV8_1
        if (index != -1) {                                              // we dont have a subaddress to send
            I2Cx->DR = _write_p[index++];
            if (_bytes == index)                                         // we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        } else {
            index++;
            I2Cx->DR = _reg;                                           // send the subaddress
            if (_reading || !_bytes)                                      // if receiving or sending 0 _bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }

    if (index == _bytes + 1) {                                           // we have completed the current job
        subaddress_sent = 0;                                            // reset this here
        if (final_stop) {                                               // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        }
        if (_status != NULL){
            (*_status) = I2C_JOB_COMPLETE;                               // Update _status
        }
        _busy = false;
    }
}

// ===============================================================================================

void HardwareWire::begin(uint8_t bus)
{
    I2CDevice index = (bus == 2) ? I2CDEV_2 : I2CDEV_1;

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

    if (index > I2CDEV_MAX)
        index = I2CDEV_MAX;

    // Turn on peripheral clock, save device and index
    I2Cx = i2cHardwareMap[index].dev;
    I2Cx_index = index;
    RCC_APB1PeriphClockCmd(i2cHardwareMap[index].peripheral, ENABLE);

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    i2cUnstick();

    // Init I2C peripheral
    I2C_DeInit(I2Cx);
    I2C_StructInit(&i2c);

    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Enable EVT and ERR interrupts - they are enabled by the first request
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Cmd(I2Cx, ENABLE);
    I2C_Init(I2Cx, &i2c);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = i2cHardwareMap[index].er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = i2cHardwareMap[index].ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&nvic);

    // write zeros to the buffer, and set all the indexes to zero
    memset(i2c_buffer, 0, I2C_BUFFER_SIZE*sizeof(i2cJob_t));
    i2c_buffer_count = 0;
    i2c_buffer_head = 0;
    i2c_buffer_tail = 0;
}


void HardwareWire::beginTransmission(uint8_t address)
{
    this->addr = address << 1;

    this->reg  = 0x00;
    this->data = 0x00;
}

uint8_t HardwareWire::write(uint8_t value)
{
    if (this->reg) {
        this->data = value;
    }
    else {
        this->reg = value;
    }

    return 1; // one byte "written"
}

uint8_t HardwareWire::endTransmission(bool stop)
{
    // This is done before a read request
    if (!stop) return 0; // "success"

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    _reg     = this->reg;
    _writing = 1;
    _reading = 0;
    _write_p = &this->data;
    _read_p  = &this->data;
    _bytes   = 1;
    _busy    = true;
    _error   = false;

    if (!I2Cx)
        return 1;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & 0x0100)) {                                    // ensure sending a start
            while (I2Cx->CR1 & 0x0200 && --timeout > 0) {
                ;    // wait for any stop to finish sending
            }
            if (timeout == 0)
                return 2;
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    _error = false;

    timeout = I2C_DEFAULT_TIMEOUT;

    while (_busy && --timeout > 0) {
        ;
    }
    if (timeout == 0)
        return 3;

    return 0; // success
}

uint8_t HardwareWire::requestFrom(uint8_t address, uint8_t len)
{
    this->avail = 0;
    this->bufpos = 0;

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    _addr = address << 1;
    _reg = this->reg;
    _writing = 0;
    _reading = 1;
    _read_p = this->buffer;
    _write_p = this->buffer;
    _bytes = len;
    _busy = true;
    _error = false;

    if (!I2Cx)
        return 0;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & 0x0100)) {                                    // ensure sending a start
            while (I2Cx->CR1 & 0x0200 && --timeout > 0) {
                ;    // wait for any stop to finish sending
            }
            if (timeout == 0)
                return 0;
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (_busy && --timeout > 0) {
        ;
    }

    this->avail = len;

    return (timeout == 0) ? 0 : len;
}

uint8_t HardwareWire::available(void)
{
    return this->avail;
}

uint8_t HardwareWire::read(void)
{
    this->avail--;

    return this->buffer[this->bufpos++];
}

} // extern "C"

HardwareWire Wire;

