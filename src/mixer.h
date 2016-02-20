#pragma once

void mixerInit(int16_t * motor_disarmed);

void mixerWriteMotors(
        int16_t * motors,
        int16_t * motor_disarmed, 
        uint16_t * rcData, 
        int16_t * rcCommand, 
        int16_t * axisPID, 
        bool armed);
