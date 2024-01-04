#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    bool hal_isInInterrupt(void);

    void systemWaitStart(void);

#ifndef ARDUINO
    void delay(const uint32_t msec);
    void delayMicroseconds(const uint32_t usec);
    uint64_t micros(void);
#endif

#ifdef __cplusplus
}
#endif

