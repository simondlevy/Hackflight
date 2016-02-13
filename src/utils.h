#pragma once

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

int constrain(int amt, int low, int high);
void alignSensors(int16_t *src, int16_t *dest, uint8_t rotation);
