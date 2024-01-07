#include "../mixer.hpp"

void Mixer::init(void)
{
    Mixer::init(4);
}

void Mixer::run(const demands_t & demands, float motors[])
{
    auto t = demands.thrust;
    auto r = demands.roll;
    auto p = demands.pitch;
    auto y = demands.yaw;

    _uncapped[0] = t - r + p + y;
    _uncapped[1] = t - r - p - y;
    _uncapped[2] = t + r - p + y;
    _uncapped[3] = t + r + p - y;

    Mixer::cap(motors);
}


