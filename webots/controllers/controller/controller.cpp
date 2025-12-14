/* 
   C++ flight simulator support for Hackflight with custom physics plugin

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// C++
#include <map>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

// Hackflight
#include <datatypes.h>
#include <simulator/dynamics.hpp>
#include <setpoint/lidar.hpp>

// Webots
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/range_finder.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

// ===========================================================================

static const uint8_t LIDAR_DISPLAY_SCALEUP = 32;

static WbDeviceTag _lidar;

static void showLidar(
        const int16_t * distance_mm,
        const uint16_t width,
        const uint16_t height) 
{
    (void)distance_mm;

    const uint16_t new_width = width * LIDAR_DISPLAY_SCALEUP;
    const uint16_t new_height = height * LIDAR_DISPLAY_SCALEUP;

    cv::Mat img = cv::Mat::zeros(new_height, new_width, CV_8UC1);

    for (uint8_t j=0; j<height; ++j) {

        for (uint8_t k=0; k<width; ++k) {

            const auto d = distance_mm[k * width + j];

            cv::rectangle(img,
                    cv::Point(k*LIDAR_DISPLAY_SCALEUP,
                        j*LIDAR_DISPLAY_SCALEUP),
                    cv::Point((k+1)*LIDAR_DISPLAY_SCALEUP,
                        (j+1)*LIDAR_DISPLAY_SCALEUP),
                    d == -1 ? 255 : (uint8_t)(d / 4000.f * 255), 
                    -1);
        }
    }

    cv::imshow("lidar", img);

    cv::waitKey(1);
}

static void readLidar(int16_t * distance_mm) 
{
    const int width = wb_range_finder_get_width(_lidar);
    const int height = wb_range_finder_get_height(_lidar);

    const float * image = wb_range_finder_get_range_image(_lidar);

    for (int j=0; j<height; ++j) {

        for (int k=0; k<width; ++k) {

            const float distance_m =
                wb_range_finder_image_get_depth( image, width, j, k);

            distance_mm[j*8+k] = isinf(distance_m) ? -1 :
                (int16_t)(1000 * distance_m);
        }
    }
}

// ===========================================================================

static const float ZDIST_HOVER_INIT_M = 0.4;
static const float ZDIST_HOVER_MAX_M = 1.0;
static const float ZDIST_HOVER_MIN_M = 0.2;
static const float ZDIST_HOVER_INC_MPS = 0.2;

typedef enum {

    JOYSTICK_NONE,
    JOYSTICK_UNRECOGNIZED,
    JOYSTICK_RECOGNIZED

} joystickStatus_e;

typedef enum {

    TOGGLE_HOVER,
    TOGGLE_AUTO

} toggle_e;

typedef struct {

    int8_t throttle;
    int8_t roll;
    int8_t pitch;
    int8_t yaw;

} joystick_t;

static WbDeviceTag _emitter;
static WbDeviceTag _gps;

static double _timestep;

static float _zdist;

static double _start_x, _start_y, _start_z;

static void climb(const float rate)
{
    const float time_curr = wb_robot_get_time();

    static float _time_prev;

    const float dt = _time_prev > 0 ? time_curr - _time_prev : 0;

    _time_prev = time_curr;

    _zdist = std::min(std::max(
                _zdist + rate * ZDIST_HOVER_INC_MPS * dt,
                ZDIST_HOVER_MIN_M), ZDIST_HOVER_MAX_M);
}

static void getSetpointFromKey(const int key, siminfo_t & siminfo)
{
    switch (key) {

        case WB_KEYBOARD_UP:
            siminfo.setpoint.pitch = +1.0;
            break;

        case WB_KEYBOARD_DOWN:
            siminfo.setpoint.pitch = -1.0;
            break;

        case WB_KEYBOARD_RIGHT:
            siminfo.setpoint.roll = +1.0;
            break;

        case WB_KEYBOARD_LEFT:
            siminfo.setpoint.roll = -1.0;
            break;

        case 'Q':
            siminfo.setpoint.yaw = -0.5;
            break;

        case 'E':
            siminfo.setpoint.yaw = +0.5;
            break;

        case 'W':
            climb(+1);
            break;

        case 'S':
            climb(-1);
            break;
    }

}

static void switchMode(flightMode_t & mode, const toggle_e toggle)
{
    switch (mode) {

        case MODE_IDLE:
            mode = toggle == TOGGLE_HOVER ? MODE_HOVERING : mode;
            break;

        case MODE_HOVERING:
            mode = 
                toggle == TOGGLE_HOVER ?  MODE_LANDING :
                toggle == TOGGLE_AUTO ?  MODE_AUTONOMOUS :
                mode;
            break;

        case MODE_AUTONOMOUS:
            mode = 
                toggle == TOGGLE_AUTO ?  MODE_HOVERING :
                mode;
            break;

        default:
            break;
    }
}

static void checkKeyboardToggle(
        const int key,
        const int target,
        const toggle_e toggle,
        bool & key_was_down,
        flightMode_t & flightMode) 
{
    if (key == target) {
        if (!key_was_down) {
            key_was_down = true;
            switchMode(flightMode, toggle);
        }
    }
}

static void getSimInfoFromKeyboard(
        siminfo_t & siminfo, flightMode_t & flightMode)
{
    static bool _enter_was_down;
    static bool _spacebar_was_down;

    const int key = wb_keyboard_get_key();

    if (key == -1 ) {
        _enter_was_down = false;
        _spacebar_was_down = false;
    }

    checkKeyboardToggle(key, 4, TOGGLE_HOVER, _enter_was_down, flightMode);

    checkKeyboardToggle(key, 32, TOGGLE_AUTO, _spacebar_was_down, flightMode);

    if (flightMode == MODE_HOVERING) {

        getSetpointFromKey(key, siminfo);
    }

    siminfo.flightMode = flightMode;
}

static void checkButtonToggle(
        const int button,
        const int target,
        const toggle_e toggle,
        bool & button_was_down,
        flightMode_t & flightMode)
{
    if (button == target) {
        button_was_down = true;
    }
    else {
        if (button_was_down) {
            switchMode(flightMode, toggle);
        }
        button_was_down = false;
    }
}

static std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

    {"Logitech Gamepad F310", joystick_t {-2,  4, -5, 1 } },

    {"Microsoft X-Box 360 pad", joystick_t {-2,  4, -5, 1 } }
};

static joystick_t getJoystickInfo() 
{
    return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
}

static float normalizeJoystickAxis(const int32_t rawval)
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
    return normalizeJoystickAxis(readJoystickRaw(index));
}

static void getSimInfoFromJoystick(siminfo_t & siminfo, flightMode_t & flightMode)
{
    static bool _hover_button_was_down;
    static bool _auto_button_was_down;

    auto axes = getJoystickInfo();

    const auto button = wb_joystick_get_pressed_button();

    checkButtonToggle(button, 5, TOGGLE_HOVER, _hover_button_was_down, flightMode);

    checkButtonToggle(button, 4, TOGGLE_AUTO, _auto_button_was_down, flightMode);

    siminfo.flightMode = flightMode;

    if (siminfo.flightMode == MODE_HOVERING) {

        siminfo.setpoint.pitch = readJoystickAxis(axes.pitch);
        siminfo.setpoint.roll = readJoystickAxis(axes.roll);
        siminfo.setpoint.yaw = readJoystickAxis(axes.yaw);

        climb(readJoystickAxis(axes.throttle));
    }
}



joystickStatus_e getJoystickStatus(void)
{
    auto mode = JOYSTICK_RECOGNIZED;

    auto joyname = wb_joystick_get_model();

    // No joystick
    if (joyname == NULL) {

        static bool _didWarn;

        if (!_didWarn) {
            puts("Using keyboard instead:\n");
            puts("- Use Enter to take off and land\n");
            puts("- Use W and S to go up and down\n");
            puts("- Use arrow keys to move horizontally\n");
            puts("- Use Q and E to change heading\n");
        }

        _didWarn = true;

        mode = JOYSTICK_NONE;
    }

    // Joystick unrecognized
    else if (JOYSTICK_AXIS_MAP.count(joyname) == 0) {

        mode = JOYSTICK_UNRECOGNIZED;
    }

    return mode;
}

static void reportJoystick(void)
{
    printf("Unrecognized joystick '%s' with axes ",
            wb_joystick_get_model()); 

    for (uint8_t k=0; k<wb_joystick_get_number_of_axes(); ++k) {

        printf("%2d=%+6d |", k+1, wb_joystick_get_axis_value(k));
    }
}

static void sendSimInfo(siminfo_t & siminfo)
{
    const double * xyz = wb_gps_get_values(_gps);

    if (_start_x == 0) {
        _start_x = xyz[0];
        _start_y = xyz[1];
        _start_z = xyz[2];
    }

    siminfo.start_x = _start_x;
    siminfo.start_y = _start_y;
    siminfo.start_z = _start_z;

    siminfo.setpoint.thrust = _zdist;
    siminfo.framerate = 1000 / _timestep;
    wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));
}

/*
static void reportLidar(int16_t * distance_mm) 
{
    for (int i=0; i<8; ++i) {
        for (int j=0; j<8; ++j) {
            const int16_t d = distance_mm[i*8+j];
            if (d < 0) {
                printf(" ---- ");
            }
            else {
                printf("%5d ", d);
            }
        }
        printf("\n \n \n");
    }
    printf("\n-----------------------------------------------\n \n");
}*/


static bool step(const setpointType_e setpointType)
{
    if (wb_robot_step(_timestep) == -1) {
        return false;
    }

    static flightMode_t _flightMode;

    siminfo_t siminfo = {};

    int16_t lidar_distance_mm[64] = {};

    readLidar(lidar_distance_mm);

    showLidar(lidar_distance_mm, 8, 8);

    //reportLidar(lidar_distance_mm);

    switch (getJoystickStatus()) {

        case JOYSTICK_RECOGNIZED:
            getSimInfoFromJoystick(siminfo, _flightMode);
            break;

        case JOYSTICK_UNRECOGNIZED:
            reportJoystick();
            // fall thru

        default:
            getSimInfoFromKeyboard(siminfo, _flightMode);
    }

    if (setpointType == SETPOINT_LIDAR) {
        Lidar::getSetpoint(8, 8, lidar_distance_mm, siminfo.setpoint);
    }

    // On descent, switch mode to idle when close enough to ground
    const auto z = wb_gps_get_values(_gps)[2] - _start_z; 
    if (_flightMode == MODE_LANDING && z <= Dynamics::ZMIN) {
        _flightMode = MODE_IDLE;
    }

    sendSimInfo(siminfo);

    return true;
}

static void animateMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);
    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction * 60);
}

int main(int argc, char ** argv) 
{
    cv::namedWindow("lidar");

    const std::string arg = std::string(argc < 2 ? "human" : argv[1]);

    setpointType_e setpointType = SETPOINT_HUMAN;

    if (arg == "lidar") {
    }
    else if (arg == "human") {
    }
    else {
        printf("Unrecognized setpoint '%s'; defaulting to human\n", arg.c_str());
    }

    wb_robot_init();

    _timestep = wb_robot_get_basic_time_step();

    _emitter = wb_robot_get_device("emitter");

    _lidar = wb_robot_get_device("range-finder");
    wb_range_finder_enable(_lidar, _timestep);

    _gps = wb_robot_get_device("gps");
    wb_gps_enable(_gps, _timestep);

    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, _timestep);

    wb_keyboard_enable(_timestep);

    animateMotor("motor1", -1);
    animateMotor("motor2", +1);
    animateMotor("motor3", +1);
    animateMotor("motor4", -1);

    wb_joystick_enable(_timestep);

    _zdist = ZDIST_HOVER_INIT_M;

    while (true) {

        if (!step(setpointType)) {
            break;
        }
    }

    wb_robot_cleanup();

    return 0;
}
