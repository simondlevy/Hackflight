/*
   Buffer for serial comms

   MIT License
 */

#pragma once

typedef struct {

    uint8_t byte00;
    uint8_t byte01;
    uint8_t byte02;
    uint8_t byte03;
    uint8_t byte04;
    uint8_t byte05;
    uint8_t byte06;
    uint8_t byte07;
    uint8_t byte08;
    uint8_t byte09;
    uint8_t byte10;
    uint8_t byte11;
    uint8_t byte12;
    uint8_t byte13;
    uint8_t byte14;
    uint8_t byte15;
    uint8_t byte16;
    uint8_t byte17;
    uint8_t byte18;
    uint8_t byte19;
    uint8_t byte20;
    uint8_t byte21;
    uint8_t byte22;
    uint8_t byte23;
    uint8_t byte24;
    uint8_t byte25;
    uint8_t byte26;
    uint8_t byte27;
    uint8_t byte28;
    uint8_t byte29;
    uint8_t byte30;
    uint8_t byte31;

} buffer_t;

static void _set(uint8_t & oldval, uint8_t index, uint8_t target, uint8_t newval, bool ready)
{
    oldval = ready && (index == target) ? newval : oldval;
}

void setbuff(buffer_t & buff, bool ready, uint8_t index, uint8_t value)
{
    _set(buff.byte00, index, value, 0, ready);
    _set(buff.byte01, index, value, 1, ready);
    _set(buff.byte02, index, value, 2, ready);
    _set(buff.byte03, index, value, 3, ready);
    _set(buff.byte04, index, value, 4, ready);
    _set(buff.byte05, index, value, 5, ready);
    _set(buff.byte06, index, value, 6, ready);
    _set(buff.byte07, index, value, 7, ready);
    _set(buff.byte08, index, value, 8, ready);
    _set(buff.byte09, index, value, 9, ready);
    _set(buff.byte10, index, value, 10, ready);
    _set(buff.byte11, index, value, 11, ready);
    _set(buff.byte12, index, value, 12, ready);
    _set(buff.byte13, index, value, 13, ready);
    _set(buff.byte14, index, value, 14, ready);
    _set(buff.byte15, index, value, 15, ready);
    _set(buff.byte16, index, value, 16, ready);
    _set(buff.byte17, index, value, 17, ready);
    _set(buff.byte18, index, value, 18, ready);
    _set(buff.byte19, index, value, 19, ready);
    _set(buff.byte20, index, value, 20, ready);
    _set(buff.byte21, index, value, 21, ready);
    _set(buff.byte22, index, value, 22, ready);
    _set(buff.byte23, index, value, 23, ready);
    _set(buff.byte24, index, value, 24, ready);
    _set(buff.byte25, index, value, 25, ready);
    _set(buff.byte26, index, value, 26, ready);
    _set(buff.byte27, index, value, 27, ready);
    _set(buff.byte28, index, value, 28, ready);
    _set(buff.byte29, index, value, 29, ready);
    _set(buff.byte30, index, value, 30, ready);
    _set(buff.byte31, index, value, 31, ready);
}

uint8_t getbuff(buffer_t & buff, uint8_t index)
{
    return index == 0 ? buff.byte00
        : index == 1  ? buff.byte01 
        : index == 2  ? buff.byte02 
        : index == 3  ? buff.byte03 
        : index == 4  ? buff.byte04 
        : index == 5  ? buff.byte05 
        : index == 6  ? buff.byte06 
        : index == 7  ? buff.byte07 
        : index == 8  ? buff.byte08 
        : index == 9  ? buff.byte09 
        : index == 10 ? buff.byte10 
        : index == 11 ? buff.byte11 
        : index == 12 ? buff.byte12 
        : index == 13 ? buff.byte13 
        : index == 14 ? buff.byte14 
        : index == 15 ? buff.byte15 
        : index == 16 ? buff.byte16 
        : index == 17 ? buff.byte17 
        : index == 18 ? buff.byte18 
        : index == 19 ? buff.byte19 
        : index == 20 ? buff.byte20 
        : index == 21 ? buff.byte21 
        : index == 22 ? buff.byte22 
        : index == 23 ? buff.byte23 
        : index == 24 ? buff.byte24 
        : index == 25 ? buff.byte25 
        : index == 26 ? buff.byte26 
        : index == 27 ? buff.byte27 
        : index == 28 ? buff.byte28 
        : index == 29 ? buff.byte29 
        : index == 30 ? buff.byte30 
        : index == 31 ? buff.byte31 
        : 0;
}
