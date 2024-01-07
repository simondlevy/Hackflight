#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    bool hal_isInInterrupt(void);

    bool serial1Read(uint8_t * byte);

    void systemWaitStart(void);

#ifdef __cplusplus
}
#endif

