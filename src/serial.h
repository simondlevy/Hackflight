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

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

#include "time.h"

typedef enum {
    SERIAL_PORT_ALL = -2,
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_USART7,
    SERIAL_PORT_USART8,
    SERIAL_PORT_UART9,
    SERIAL_PORT_USART10,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_LPUART1 = 40,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_LPUART1,
} serialPortIdentifier_e;


typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_400000,
    BAUD_460800,
    BAUD_500000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000,
    BAUD_COUNT
} baudRate_e;

typedef enum {
    MODE_RX = 1 << 0,
    MODE_TX = 1 << 1,
    MODE_RXTX = MODE_RX | MODE_TX
} portMode_e;

typedef enum {
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3,

    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    SERIAL_BIDIR_OD        = 0 << 4,
    SERIAL_BIDIR_PP        = 1 << 4,
    SERIAL_BIDIR_NOPULL    = 1 << 5, // disable pulls in BIDIR RX mode
    SERIAL_BIDIR_PP_PD     = 1 << 6, // PP mode, normall inverted, but with PullDowns, to fix 
    // SA after bidir issue fixed (#10220)

} portOptions_e;


typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0),  // 1
    FUNCTION_TELEMETRY_FRSKY_HUB = (1 << 2),  // 4
    FUNCTION_TELEMETRY_HOTT      = (1 << 3),  // 8
    FUNCTION_TELEMETRY_LTM       = (1 << 4),  // 16
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
    FUNCTION_RX_SERIAL           = (1 << 6),  // 64
    FUNCTION_BLACKBOX            = (1 << 7),  // 128
    FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
    FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
    FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
    FUNCTION_TELEMETRY_IBUS      = (1 << 12), // 4096
    FUNCTION_VTX_TRAMP           = (1 << 13), // 8192
    FUNCTION_RCDEVICE            = (1 << 14), // 16384
    FUNCTION_LIDAR_TF            = (1 << 15), // 32768
    FUNCTION_FRSKY_OSD           = (1 << 16), // 65536
} serialPortFunction_e;

#if defined(__cplusplus)
extern "C" {
#endif

    typedef void (*serialReceiveCallbackPtr)
        (uint8_t data, void *rxCallbackData, uint32_t usec);

    void serialInit(serialPortIdentifier_e serialPortToDisable);

    bool serialIsTransmitBufferEmpty(void * port);

    void * serialOpenPort(
            serialPortIdentifier_e identifier,
            serialPortFunction_e function,
            serialReceiveCallbackPtr rxCallback,
            void *rxCallbackData,
            uint32_t baudRate,
            portMode_e mode,
            portOptions_e options);

    void serialOpenPortDsmx(
            serialPortIdentifier_e identifier,
            serialReceiveCallbackPtr callback,
            void * data);

    void serialOpenPortSbus(
            serialPortIdentifier_e identifier,
            serialReceiveCallbackPtr callback,
            void * data);

    void * serialOpenPortUsb(void);

    uint8_t serialRead(void  * port);

    uint32_t serialBytesAvailable(void * port);

    void serialSetDebugPort(void * port);

    void serialWrite(void * port, uint8_t c);

    void serialWriteBuf(void * port, const uint8_t *data, uint32_t count);
#if defined(__cplusplus)
}
#endif
