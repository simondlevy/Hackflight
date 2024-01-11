#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    //int consolePrintf(const char * fmt, ...);

    bool hal_isInInterrupt(void);

    void systemWaitStart(void);

#ifdef __cplusplus
}
#endif

