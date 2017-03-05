#include <hackflight.hpp>
#include "naze.hpp"

int main()
{
    hf::Hackflight h;
    h.init(new hf::Naze());

    while (true) {
        h.update();
    }
}
