#pragma once

typedef enum {
    ADC_BATTERY = 0,
    ADC_EXTERNAL_PAD = 1,
    ADC_EXTERNAL_CURRENT = 2,
    ADC_RSSI = 3,
    ADC_CHANNEL_MAX = 4
} AdcChannel;

void adcInit(int hwrev);

uint16_t adcGetChannel(uint8_t channel);
