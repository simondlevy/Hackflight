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

    motors[0] = t - r + p + y;
    motors[1] = t - r - p - y;
    motors[2] = t + r - p + y;
    motors[3] = t + r + p - y;
}


