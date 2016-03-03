/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include <breezystm32.h>

#include <string.h>

#include "mw.h"
#include "config.h"
#include "utils.h"

static int16_t motorsDisarmed[4];

static int16_t motors[4];

typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

void mixerInit(void)
{
    int i;
    for (i = 0; i < 4; i++)
        motorsDisarmed[i] = CONFIG_MINCOMMAND;
}

void mixerWriteMotors(bool armed)
{
    int16_t maxMotor;
    uint32_t i;

    // prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));

    for (i = 0; i < 4; i++)
        motors[i] = rcCommand[THROTTLE] * mixerQuadX[i].throttle + axisPID[PITCH] * mixerQuadX[i].pitch + 
            axisPID[ROLL] * mixerQuadX[i].roll + -CONFIG_YAW_DIRECTION * axisPID[YAW] * mixerQuadX[i].yaw;

    maxMotor = motors[0];
    for (i = 1; i < 4; i++)
        if (motors[i] > maxMotor)
            maxMotor = motors[i];
    for (i = 0; i < 4; i++) {
        if (maxMotor > CONFIG_MAXTHROTTLE)     
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors[i] -= maxMotor - CONFIG_MAXTHROTTLE;

        motors[i] = constrain(motors[i], CONFIG_MINTHROTTLE, CONFIG_MAXTHROTTLE);
        if ((rcData[THROTTLE]) < CONFIG_MINCHECK) {
            motors[i] = CONFIG_MINTHROTTLE;
        } 
        if (!armed) {
            motors[i] = motorsDisarmed[i];
        }
    }

    for (i = 0; i < 4; i++)
        pwmWriteMotor(i, motors[i]);
}

uint16_t mixerGetMotor(uint8_t i)
{
    return motors[i];
}

void mixerSetMotor(uint8_t i, uint16_t value)
{
    motorsDisarmed[i] = value;
}
