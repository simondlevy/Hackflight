#pragma once

#include "datatypes.h"

static throttleStatus_e rxCalculateThrottleStatus(float * rcData)
{
    return (rcData[THROTTLE] < 1050) ?  THROTTLE_LOW : THROTTLE_HIGH;
}
