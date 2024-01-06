#pragma once

#include <stdbool.h>
#include <stdint.h>

static const uint32_t WATCHDOG_RESET_PERIOD_MS = 80;

void watchdogInit(void);
bool watchdogNormalStartTest(void);
void watchdogReset(void);

