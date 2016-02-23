#include "board.h"
#include "mw.h"

static bool isTelemetryConfigurationValid = false; // flag used to avoid repeated configuration checks
void checkTelemetryState(void);

void initTelemetry(void)
{
    checkTelemetryState();
}

static bool telemetryEnabled = false;

bool determineNewTelemetryEnabledState(void)
{
    return armed;
}

bool shouldChangeTelemetryStateNow(bool newState)
{
    return newState != telemetryEnabled;
}

void checkTelemetryState(void)
{
    if (!isTelemetryConfigurationValid) {
        return;
    }

    bool newEnabledState = determineNewTelemetryEnabledState();

    if (!shouldChangeTelemetryStateNow(newEnabledState)) {
        return;
    }

    telemetryEnabled = newEnabledState;
}

void handleTelemetry(void)
{
    if (!isTelemetryConfigurationValid || !determineNewTelemetryEnabledState())
        return;
}
