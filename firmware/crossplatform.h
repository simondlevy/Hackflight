// support simulator on Windows

#pragma once

#ifdef _WIN32
#define M_PI 3.14159265358979323846
#define lrintf(x) (float)(int)((x)+0.5)
#else
#include <stdbool.h>
#endif
