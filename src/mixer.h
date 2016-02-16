#pragma once

extern int16_t motor[4];

void mixerInit(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(int16_t * rcCommand);
