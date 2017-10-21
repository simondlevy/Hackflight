#include <math.h>
#include <time.h>

#include <hackflight.hpp>
#include <models/3dfly.hpp> // arbitrary
#include <receivers/sim.hpp>

namespace hf {

class NullBoard : public Board {

    virtual void init(Config& config) override
    {
        (void)config;
    }

    virtual void delayMilliseconds(uint32_t msec) override
    {
    }

    virtual void getImu(float eulerAnglesRadians[3], int16_t gyroRaw[3]) override
    {
        for (int k=0; k<3; ++k) {
            eulerAnglesRadians[k] = 0;
            gyroRaw[k] = 0;
        }
    }

    virtual uint64_t getMicros() override
    {
        struct timespec t;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
        return 1000000*t.tv_sec + t.tv_nsec/1000;
    }

    virtual void writeMotor(uint8_t index, float value) override
    {
        (void)index;
        (void)value;
    }


}; // class NullBoard

} // namespace hf

int main(int argc, char ** argv)
{
    hf::Hackflight h;
    hf::NullBoard  board;
    hf::Controller controller;
    hf::ThreeDFly  model;

    h.init(&board, &controller, &model);

    while (true) {
        h.update();
    }

    return 0;
}
