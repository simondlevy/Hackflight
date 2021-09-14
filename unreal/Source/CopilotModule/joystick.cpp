/*
 * Windows support for joysticks and other game controllers
 *
 * Copyright (C) 2021 Simon D. Levy
 *
 * MIT License
 */


#include "Joystick.h"

#undef TEXT
#include <shlwapi.h>
#include "joystickapi.h"

bool IJoystick::isValidJoystick(int joystick_id, uint16_t & product_id)
{
    JOYCAPS joycaps = {};

    if (joyGetDevCaps(joystick_id,
                &joycaps, sizeof(joycaps)) == JOYERR_NOERROR) {
        product_id = joycaps.wPid;
        return true;
    }

    return false;
}

void IJoystick::readJoystick(
        int joystick_id,
        uint32_t & xpos,
        uint32_t & ypos,
        uint32_t & zpos, 
        uint32_t & rpos,
        uint32_t & upos,
        uint32_t & vpos, 
        uint8_t & buttons)
{   
    JOYINFOEX joyState;
    joyState.dwSize = sizeof(joyState);
    joyState.dwFlags =
        JOY_RETURNALL | JOY_RETURNPOVCTS | JOY_RETURNCENTERED | JOY_USEDEADZONE;
    joyGetPosEx(joystick_id, &joyState);

    xpos = joyState.dwXpos; 
    ypos = joyState.dwYpos;
    zpos = joyState.dwZpos; 
    rpos = joyState.dwRpos; 
    upos = joyState.dwUpos;
    vpos = joyState.dwVpos; 

    buttons = (uint8_t)joyState.dwButtons;

}

