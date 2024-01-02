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

} joystickAxes_t;

static std::map<std::string, joystickAxes_t> JOYSTICK__AXIS_MAP = {

    { "MY-POWER CO.,LTD. 2In1 USB Joystick", joystickAxes_t {-1, -2, -3, 0 } },
    { "SHANWAN Android Gamepad",             joystickAxes_t {-1, -2, -3, 0 } },
    { "FrSky FrSky Simulator",               joystickAxes_t { 0,  1,  2, 3 } },
    { "Horizon Hobby SPEKTRUM RECEIVER",     joystickAxes_t { 1,  2,  3, 0 } },
};

static float scaleJoystickAxis(const int32_t rawval)
{
    return 2.0f * rawval / UINT16_MAX; 
}

static int32_t readJoystickRaw(const int8_t index)
{
    const auto axis = abs(index);

    const auto sign = index < 0 ? -1 : +1;

    return sign * wb_joystick_get_axis_value(axis);
}

static float readJoystickAxis(const int8_t index)
{
    return scaleJoystickAxis(readJoystickRaw(index));
}

// Starting at low throttle (as we should) produces an initial stick value of
// zero.  So we check for this and adjust as needed.
static float readJoystickThrust(const joystickAxes_t axes)
{
    static bool didMoveStick;

    const auto raw = readJoystickRaw(axes.thrust);

    if (raw != 0) {
        didMoveStick = true;
    }

    return didMoveStick ? scaleJoystickAxis(raw) : -1;
}

static demands_t readJoystick(void)
{
    auto axes = JOYSTICK__AXIS_MAP[wb_joystick_get_model()];

    return demands_t {

        readJoystickThrust(axes), 
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

static bool haveJoystick(void)
{
    bool have = true;

    const char * joyname = wb_joystick_get_model();

    if (joyname == NULL || JOYSTICK__AXIS_MAP.count(joyname) == 0) {

        static bool didWarn;

        if (!didWarn) {
            puts("Using keyboard instead:\n");
            puts("- Use arrow keys to move in the horizontal plane\n");
            puts("- Use Q and E to rotate around yaw\n");
            puts("- Use W and S to go up and down\n");
        }

        didWarn = true;

        have = false;
    }

    return have;
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
    return haveJoystick() ? readJoystick() : readKeyboard();
}
