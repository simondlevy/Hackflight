#pragma once

/* A LED sequence is made of a list of actions. Each action contains the new
 * state of the LED and either a time to wait before executing the next action
 * or a command LOOP or STOP.
 *
 * The sequences are stored in a list by priority order ([0] is the highest
 * priority). The list ordered by priority is defined at the beginning of
 * ledseq.c
 *
 * Each sequence effects only one LED. For each LED only the runnable sequence
 * with the highest priority is run.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

void ledseqInit(void);
void ledseqSetChargeLevel(const float chargeLevel);
void ledseqShowBattery(void);
void ledseqShowCharged(void);
void ledseqShowCharging(void);
void ledseqShowCalibrated(void);
void ledseqShowFailure(void);
void ledseqShowLinkDown(void);
void ledseqShowLinkUp(void);
void ledseqShowLowPower(void);
void ledseqShowSuccess(void);
bool ledseqTest(void);




