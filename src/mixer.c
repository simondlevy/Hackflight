/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include <breezystm32.h>

#include "axes.h"
#include "mixer.h"
#include "chans.h"
#include "config.h"
#include "utils.h"

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

static motorMixer_t currentMixer[4];

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

void mixerInit(int16_t * motor_disarmed)
{
    int i;

    for (i = 0; i < 4; i++) {
        currentMixer[i] = mixerQuadX[i];
        motor_disarmed[i] = CONFIG_MINCOMMAND;
    }
}

void mixerWriteMotors(
        int16_t * motors, 
        int16_t * motor_disarmed, 
        uint16_t * rcData, 
        int16_t * rcCommand, 
        int16_t * axisPID, 
        bool armed)
{
    int16_t maxMotor;
    uint32_t i;

    // prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));

    for (i = 0; i < 4; i++)
        motors[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + 
            axisPID[ROLL] * currentMixer[i].roll + -CONFIG_YAW_DIRECTION * axisPID[YAW] * currentMixer[i].yaw;

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
            motors[i] = motor_disarmed[i];
        }
    }

    for (i = 0; i < 4; i++)
        pwmWriteMotor(i, motors[i]);
}
