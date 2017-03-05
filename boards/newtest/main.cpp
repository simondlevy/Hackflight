#include <hackflight.hpp>
#include "mysimboard.hpp"

int main()
{
    hf::Hackflight h;
    h.init(new hf::SimBoard());

    uint32_t count = 0;

    while (true) {
        h.update();
    }
}
