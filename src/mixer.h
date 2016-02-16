#pragma once

extern int16_t motor[4];
extern int16_t rcCommand[4];

void mixerInit(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(void);
