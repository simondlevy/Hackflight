#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    void usecTimerInit(void);

    void delay(const uint32_t msec);

    void delayMicroseconds(const uint32_t usec);

    uint64_t micros(void);

#ifdef __cplusplus
}
#endif
