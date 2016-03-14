#include <breezystm32.h>

#include "board.h"

void board_writeMotor(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, value);
}
