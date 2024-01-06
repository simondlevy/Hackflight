#include <commander.hpp>
#include <openloop.hpp>

void OpenLoop::getDemands(demands_t & demands, uint32_t & timestamp, bool & inHoverMode)
{
    extern Commander commander;

    commander.getDemands(demands, timestamp, inHoverMode);
}
