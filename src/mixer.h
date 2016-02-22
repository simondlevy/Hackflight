#pragma once

void mixerInit(int16_t * motorDisarmed);

void mixerWriteMotors(
        int16_t * motors,
        int16_t * motorDisarmed, 
        uint16_t * rcData, 
        int16_t * rcCommand, 
        int16_t * axisPID, 
        bool armed);
