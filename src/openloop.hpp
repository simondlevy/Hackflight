#pragma once

#include <stdint.h>

#include <datatypes.h>

class OpenLoop {

    public:

        void getDemands(demands_t & demands, uint32_t & timestamp, bool & inHoverMode);
};
