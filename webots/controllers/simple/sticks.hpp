/**
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include <map>
#include <string>

#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/robot.h>

typedef struct {

    int8_t thrust;
    int8_t roll;
    int8_t pitch;
    int8_t yaw;
    int8_t hover;

} joystickAxes_t;

typedef enum {

    JOYSTICK_NONE,
    JOYSTICK_UNRECOGNIZED,
    JOYSTICK_RECOGNIZED

} joystickStatus_e;

static std::map<std::string, joystickAxes_t> JOYSTICK_AXIS_MAP = {

    //                                                        T   R   P  Y   H
    // Linux   
    { "MY-POWER CO.,LTD. 2In1 USB Joystick", joystickAxes_t {-2,  3, -4, 1,  5 } }, 
    { "SHANWAN Android Gamepad",             joystickAxes_t {-2,  3, -4, 1,  7 } },
    { "Logitech Logitech Extreme 3D",        joystickAxes_t {-4,  1, -2, 3,  0 }  },
    { "Logitech Gamepad F310",               joystickAxes_t {-2,  4, -5, 1,  5 } },
    { "FrSky FrSky Simulator",               joystickAxes_t { 1,  2,  3, 4,  0 } },
    { "Horizon Hobby SPEKTRUM RECEIVER",     joystickAxes_t { 2,  3,  4, 1,  0 } },

    // Windows
    { "2In1 USB Joystick",                   joystickAxes_t {-1,  4, -3, 2, 5 } },
    { "Controller (XBOX 360 For Windows)",   joystickAxes_t {-1,  4, -3, 2, 5 } },
    { "Controller (Gamepad F310)",           joystickAxes_t {-1,  4, -3, 2, 5 } },
    { "Logitech Extreme 3D",                 joystickAxes_t { 0,  2, -1, 3, 0 } },
    { "FrSky Simulator",                     joystickAxes_t { 6,  5,  4, 3, 0 } },
    { "SPEKTRUM RECEIVER",                   joystickAxes_t { 3,  2,  1, 4, 0 } },  
};

static float scaleJoystickAxis(const int32_t rawval)
{
    return 2.0f * rawval / UINT16_MAX; 
}

static int32_t readJoystickRaw(const int8_t index)
{
    const auto axis = abs(index) - 1;
    const auto sign = index < 0 ? -1 : +1;
    return sign * wb_joystick_get_axis_value(axis);
}

static float readJoystickAxis(const int8_t index)
{
    return scaleJoystickAxis(readJoystickRaw(index));
}

static float readThrottleNormal(joystickAxes_t axes)
{
    static bool didMoveStick;

    const auto raw = readJoystickRaw(axes.thrust);

    if (raw != 0) {
        didMoveStick = true;
    }

    return didMoveStick ? scaleJoystickAxis(raw) : -1;
}

static float readThrottleExtremeWindows(void)
{
    static bool didWarn;

    if (!didWarn) {
           printf("Use trigger to climb, side-button to descend\n");
    }

    didWarn = true;

    auto button = wb_joystick_get_pressed_button();

    return button == 0 ? + 0.5 : button == 1 ? -0.5 : 0;
}

// Special handling for throttle stick: 
//
// 1. Check for Logitech Extreme Pro 3D on Windows; have to use buttons for throttle.
//
// 2. Starting at low throttle (as we should) produces an initial stick value
// of zero.  So we check for this and adjust as needed.
//
static float readJoystickThrust(const char * name, const joystickAxes_t axes)
{
    return !strcmp(name, "Logitech Extreme 3D") ? 
        readThrottleExtremeWindows() : 
        readThrottleNormal(axes);
}

static demands_t readJoystick(void)
{
    auto joyname = wb_joystick_get_model();

    auto axes = JOYSTICK_AXIS_MAP[joyname];

    return demands_t {

        readJoystickThrust(joyname, axes), 
            -readJoystickAxis(axes.roll),  // postive roll-leftward
            readJoystickAxis(axes.pitch), 
            readJoystickAxis(axes.yaw)

    };
}

static demands_t readKeyboard(void)
{
    demands_t demands = {0, 0, 0, 0};

    switch (wb_keyboard_get_key()) {

        case WB_KEYBOARD_UP:
            demands.pitch = +0.5;
            break;

        case WB_KEYBOARD_DOWN:
            demands.pitch = -0.5;
            break;

        case WB_KEYBOARD_RIGHT:
            demands.roll = -0.5;
            break;

        case WB_KEYBOARD_LEFT:
            demands.roll = +0.5;
            break;

        case 'Q':
            demands.yaw = -0.5;
            break;

        case 'E':
            demands.yaw = +0.5;
            break;

        case 'W':
            demands.thrust = +0.5;
            break;

        case 'S':
            demands.thrust = -0.5;
            break;
    }

    return demands;

}

static joystickStatus_e haveJoystick(void)
{
    auto status = JOYSTICK_RECOGNIZED;

    auto joyname = wb_joystick_get_model();

    // No joystick
    if (joyname == NULL) {

        static bool didWarn;

        if (!didWarn) {
            puts("Using keyboard instead:\n");
            puts("- Use arrow keys to move in the horizontal plane\n");
            puts("- Use Q and E to rotate around yaw\n");
            puts("- Use W and S to go up and down\n");
        }

        didWarn = true;

        status = JOYSTICK_NONE;
    }

    // Joystick unrecognized
    else if (JOYSTICK_AXIS_MAP.count(joyname) == 0) {

        status = JOYSTICK_UNRECOGNIZED;
    }

    return status;
}

static demands_t reportJoystick(void) 
{
    printf("Unrecognized joystick '%s' with axes ", wb_joystick_get_model()); 

    for (uint8_t k=0; k<wb_joystick_get_number_of_axes(); ++k) {

        printf("%2d=%+6d |", k+1, wb_joystick_get_axis_value(k));
    }

    printf(" Button pressed = %d\n", wb_joystick_get_pressed_button());

    return demands_t {0, 0, 0, 0};
}


static bool getHoverModeFromJoystick(void)
{
    auto joyname = wb_joystick_get_model();

    auto axes = JOYSTICK_AXIS_MAP[joyname];

    auto hover = axes.hover;

    // For gamepad with shoulder buttons, allow use to enter/exit hover mode;
    // for R/C/ transmitters and keyboard, always in hover mode
    return 
        hover > 0  ? 
        wb_joystick_get_pressed_button() == abs(hover) :
        true;         
}

static bool sticksInHoverMode(void)
{
    return 
        haveJoystick() == JOYSTICK_RECOGNIZED ? getHoverModeFromJoystick() : 
        true;
}

//////////////////////////////////////////////////////////////////////////////

static void sticksInit(void)
{
    const auto timestep = wb_robot_get_basic_time_step();

    wb_joystick_enable(timestep);
    wb_keyboard_enable(timestep);
}

static demands_t sticksRead(void)
{
    auto joystickStatus = haveJoystick();

    return 
        joystickStatus == JOYSTICK_RECOGNIZED ? readJoystick() : 
        joystickStatus == JOYSTICK_UNRECOGNIZED ? reportJoystick() :
        readKeyboard();
}
