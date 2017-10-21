#include <math.h>

#include <hackflight.hpp>

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
        return 0;
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
    return 0;
}
