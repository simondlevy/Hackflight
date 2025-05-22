#pragma once

#include <datatypes.h>

void runClosedLoopControl(
        const float dt,
        const bool inHoverMode,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands);
