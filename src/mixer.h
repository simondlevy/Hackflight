#pragma once

void mixerInit(int16_t motor_disarmed[4]);
void writeMotors(int16_t motor[4]);
void mixTable(int16_t * rcCommand, bool armed, int16_t motor[4], int16_t motor_disarmed[4]);
