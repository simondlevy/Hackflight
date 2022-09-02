/*
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

#include "platform.h"
#include "resource.h"
#include "io_types.h"

// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)  // TODO
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_Mode_IN,  GPIO_Speed_25MHz, 0, GPIO_PuPd_UP)

#include "io_def.h"

#if defined(__cplusplus)
extern "C" {
#endif

    bool IORead(IO_t io);
    void IOWrite(IO_t io, bool value);
    void IOHi(IO_t io);
    void IOLo(IO_t io);
    void IOToggle(IO_t io);

    void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
    void IORelease(IO_t io);  // unimplemented
    resourceOwner_e IOGetOwner(IO_t io);
    bool IOIsFreeOrPreinit(IO_t io);
    IO_t IOGetByTag(ioTag_t tag);

    void IOConfigGPIO(IO_t io, ioConfig_t cfg);
    void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);

    void ioInitGlobal(void);

    typedef void (*IOTraverseFuncPtr_t)(IO_t io);

    void IOTraversePins(IOTraverseFuncPtr_t func);

    GPIO_TypeDef* IO_GPIO(IO_t io);

    uint16_t IO_Pin(IO_t io);

#if defined(__cplusplus)
}
#endif
