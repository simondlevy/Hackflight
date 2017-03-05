#include <hackflight.hpp>
#include <simboard.hpp>

int main()
{
    hf::Hackflight h;
    h.init(new hf::SimBoard());

    while (true) {

    }
}
