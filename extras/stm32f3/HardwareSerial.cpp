/*
   HardwareSerial.cpp : UART support for STM32F3 boards

   Copyright (C) 2018 Simon D. Levy 

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

void serialEvent1(void) __attribute__((weak));
void serialEvent2(void) __attribute__((weak));
void serialEvent3(void) __attribute__((weak));

extern "C" {

#include "platform.h"
#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "timer.h"
#include "serial.h"
#include "serial_uart.h"
#include "exti.h"
#include "bus_spi.h"

#include "HardwareSerial.h"

    // Defined in f3board.cpp
    extern serialPort_t * serial0;

    void HardwareSerial::write(uint8_t byte)
    {
        serialPort_t * port = (serialPort_t *)this->_uart;
        serialWrite(port, byte);
    }

    uint8_t HardwareSerial::available(void)
    {
        serialPort_t * port = (serialPort_t *)this->_uart;
        return serialRxBytesWaiting(port);
    }

    void HardwareSerial::flush(void)
    {
        serialPort_t * port = (serialPort_t *)this->_uart;
        while (!isSerialTransmitBufferEmpty(port));
    }

#define SERIAL_RX_BUFSIZE 256

    static uint8_t serial1_rx_buffer[SERIAL_RX_BUFSIZE];
    static uint8_t serial1_rx_index;

    static void serial_event_1(uint16_t value)
    {
        serial1_rx_buffer[serial1_rx_index] = (uint8_t)value;

        serialEvent1();

        serial1_rx_index = (serial1_rx_index + 1) % SERIAL_RX_BUFSIZE;
    }

    void HardwareSerial1::begin(uint32_t baud)
    {
        this->_uart = uartOpen(USART1, serial_event_1, baud, MODE_RX, SERIAL_NOT_INVERTED);

        serial1_rx_index = 0;
    }

    uint8_t HardwareSerial1::read(void)
    {
        return serial1_rx_buffer[serial1_rx_index];
    }

    static uint8_t serial2_rx_buffer[SERIAL_RX_BUFSIZE];
    static uint8_t serial2_rx_index;

    static void serial_event_2(uint16_t value)
    {
        serial2_rx_buffer[serial2_rx_index] = (uint8_t)value;

        serialEvent2();

        serial2_rx_index = (serial2_rx_index + 1) % SERIAL_RX_BUFSIZE;
    }

    void HardwareSerial2::begin(uint32_t baud)
    {
        this->_uart = uartOpen(USART2, serial_event_2, baud, MODE_RX, SERIAL_NOT_INVERTED);

        serial2_rx_index = 0;
    }

    uint8_t HardwareSerial2::read(void)
    {
        return serial2_rx_buffer[serial2_rx_index];
    }

    static uint8_t serial3_rx_buffer[SERIAL_RX_BUFSIZE];
    static uint8_t serial3_rx_index;

    static void serial_event_3(uint16_t value)
    {
        serial3_rx_buffer[serial3_rx_index] = (uint8_t)value;

        serialEvent3();

        serial3_rx_index = (serial3_rx_index + 1) % SERIAL_RX_BUFSIZE;
    }

    void HardwareSerial3::begin(uint32_t baud)
    {
        this->_uart = uartOpen(USART3, serial_event_3, baud, MODE_RX, SERIAL_NOT_INVERTED);

        serial3_rx_index = 0;
    }

    uint8_t HardwareSerial3::read(void)
    {
        return serial3_rx_buffer[serial3_rx_index];
    }

} // extern "C"

HardwareSerial1 Serial1;
HardwareSerial2 Serial2;
HardwareSerial3 Serial3;
