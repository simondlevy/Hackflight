/*
   Arduino support for CPPM receivers

   Copyright (C) 2022 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#define _EXTERN
#include "hackflight.h"

#include "arduino_debugger.hpp"

static const uint8_t MAX_CHANS = 8;
static const uint16_t MINPULSE  = 700;
static const uint16_t MAXPULSE  = 2250;
static const uint16_t SYNCPULSE = 7500;     

static volatile uint16_t _ppmTmp[MAX_CHANS];
static volatile uint32_t _startPulse;
static volatile uint8_t  _ppmCounter;
static volatile uint16_t _ppmError;
static volatile uint16_t _rcvr[MAX_CHANS];
static volatile bool _got_new_frame;

static void _isr(void)
{
    uint32_t stopPulse = micros();

    // clear channel interrupt flag (CHF)
    volatile uint32_t pulseWidth = stopPulse - _startPulse;

    // Error sanity check
    if (pulseWidth < MINPULSE || (pulseWidth > MAXPULSE &&
                pulseWidth < SYNCPULSE)) { _ppmError++;

        // set _ppmCounter out of range so rest and (later on) whole frame is dropped
        _ppmCounter = MAX_CHANS + 1;
    }
    if (pulseWidth >= SYNCPULSE) {
        // Verify if this is the sync pulse
        if (_ppmCounter <= MAX_CHANS) {

            // This indicates that we received a correct frame = push to the
            // "main" PPM array if we received an broken frame, it will get
            // ignored here and later get over-written
            // by new data, that will also be checked for sanity.
            for (uint8_t i = 0; i < MAX_CHANS; i++) {
                _rcvr[i] = _ppmTmp[i];             
            }
        }

        // restart the channel counter
        _ppmCounter = 0;

        _got_new_frame = true;

    } else {  
        // extra channels will get ignored here
        if (_ppmCounter < MAX_CHANS) {   
            // Store measured pulse length in us
            _ppmTmp[_ppmCounter] = pulseWidth;

            // Advance to next channel
            _ppmCounter++;
        }
    }

    // Save time at pulse start
    _startPulse = stopPulse;
}

// ------------------------------------------------------------------

static uint8_t _nchan;

void cppmStart(uint8_t pin, uint8_t nchan)
{
    _nchan = nchan;

    pinMode(pin, INPUT);

    attachInterrupt(digitalPinToInterrupt(pin), _isr, RISING);

    for (uint8_t k=0; k<MAX_CHANS; ++k) {
        _rcvr[k] = 1500;
        _ppmTmp[k] = 1500;
    }

    _ppmCounter = MAX_CHANS;
    _ppmCounter = 0;
    _ppmError = 0;
}

void cppmUpdate(void)
{
    receiverGotNewFrame = _got_new_frame;
    if (_got_new_frame) {
        _got_new_frame = false;
    }
}

void cppmGet(void)
{
    static uint16_t rcData4Values[MAX_CHANS][4], rcDataMean[MAX_CHANS];
    static uint8_t rc4ValuesIndex = 0;
    uint32_t rawRC[MAX_CHANS];

    rc4ValuesIndex++;
    if (rc4ValuesIndex == 4) rc4ValuesIndex = 0;

    for (uint8_t k=0; k<_nchan; ++k) { 
        rawRC[k] = _rcvr[k];
    }

    uint16_t rcData[MAX_CHANS] = {};

    for (uint8_t chan=0; chan < _nchan; chan++) {
        rcData4Values[chan][rc4ValuesIndex] = rawRC[chan];
        rcDataMean[chan] = 0;
        for (uint8_t a=0; a<4; a++) rcDataMean[chan] += rcData4Values[chan][a];
        rcDataMean[chan]= (rcDataMean[chan] + 2) >> 2;
        if (rcDataMean[chan] < (uint16_t)rcData[chan] - 3) {
            rcData[chan] = rcDataMean[chan] + 2;
        }
        if (rcDataMean[chan] > (uint16_t)rcData[chan] + 3) {
            rcData[chan] = rcDataMean[chan] - 2;
        }
    }

    receiverChan1 = rcData[0];
    receiverChan2 = rcData[1];
    receiverChan3 = rcData[2];
    receiverChan4 = rcData[3];
    receiverChan5 = rcData[4];
    receiverChan6 = rcData[5];
}

void debug(uint16_t c1, uint16_t c2, uint16_t c3, uint16_t c4, uint16_t c5, uint16_t c6)
{
    Debugger::printf("%04d  %04d  %04d  %04d  %04d  %04d\n", c1, c2, c3, c4, c5, c6);
}
