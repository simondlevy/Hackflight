//#include "hackflight.h"
#include "arduino_debugger.hpp"

void debug(float t, float r, float p, float y, float a1, float a2)
{
    Debugger::printf("t=%+3.3f  r=%+3.3f   p=%+3.3f   y=%+3.3f   a1=%+3.3f   a2=%+3.3f\n", t, r, p, y, a1, a2);
}
