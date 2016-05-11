extern "C" {

#include "mw.hpp"
#include "rc.hpp"

#include <strings.h>

void RC::init(void)
{
    bzero (this->dataAverage, 8*4*sizeof(int16_t));

    this->commandDelay = 0;
    this->sticks = 0;
    this->averageIndex = 0;

    for (uint8_t i = 0; i < RC_CHANS; i++)
        this->data[i] = 1502;

    for (uint8_t i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + CONFIG_RC_EXPO_8 * (i * i - 25)) * i * (int32_t)CONFIG_RC_RATE_8 / 2500;

    for (uint8_t i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - CONFIG_THR_MID_8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - CONFIG_THR_MID_8;
        if (tmp < 0)
            y = CONFIG_THR_MID_8;
        lookupThrottleRC[i] = 10 * CONFIG_THR_MID_8 + tmp * (100 - CONFIG_THR_EXPO_8 + 
                (int32_t)CONFIG_THR_EXPO_8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = CONFIG_MINTHROTTLE + (int32_t)(CONFIG_MAXTHROTTLE - CONFIG_MINTHROTTLE) * 
            lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

void RC::update(void)
{
    for (uint8_t chan = 0; chan < 8; chan++) {
    
        // get RC PWM
        this->dataAverage[chan][this->averageIndex % 4] = board_pwmRead(CONFIG_RCMAP[chan]);

        this->data[chan] = 0;

        for (uint8_t i = 0; i < 4; i++)
            this->data[chan] += this->dataAverage[chan][i];
        this->data[chan] /= 4;
    }

    this->averageIndex++;


    // check stick positions, updating command delay
    uint8_t stTmp = 0;
    for (uint8_t i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (this->data[i] > CONFIG_MINCHECK)
            stTmp |= 0x80;  // check for MIN
        if (this->data[i] < CONFIG_MAXCHECK)
            stTmp |= 0x40;  // check for MAX
    }
    if (stTmp == this->sticks) {
        if (this->commandDelay < 250)
            this->commandDelay++;
    } else
        this->commandDelay = 0;
    this->sticks = stTmp;
}

bool RC::changed(void)
{
  return this->commandDelay == 20;
}

void RC::computeExpo(void)
{
    int32_t tmp, tmp2;

    for (uint8_t axis = 0; axis < 3; axis++) {

        tmp = min(abs(this->data[axis] - CONFIG_MIDRC), 500);

        if (axis != 2) {        // ROLL & PITCH
            tmp2 = tmp / 100;
            this->command[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
        } else {                // YAW
            this->command[axis] = tmp * -CONFIG_YAW_CONTROL_DIRECTION;
        }

        if (this->data[axis] < CONFIG_MIDRC)
            this->command[axis] = -this->command[axis];
    }

    tmp = constrain(this->data[THROTTLE], CONFIG_MINCHECK, 2000);
    tmp = (uint32_t)(tmp - CONFIG_MINCHECK) * 1000 / (2000 - CONFIG_MINCHECK);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    this->command[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - 
            lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

} // computeExpo

uint8_t RC::auxState(void) 
{
    int16_t aux = this->data[4];

    return aux < 1500 ? 0 : (aux < 1700 ? 1 : 2);
}

} // extern "C"
