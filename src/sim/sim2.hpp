/*
   Webots-based flight simulator support for Hackflight using custom physics

   Copyright (C) 2024 Simon D. Levy

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


#pragma once

// C
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

// C++
#include <map>
#include <string>

// Webots
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

// Hackflight
#include <hackflight.hpp>
#include <mixers.hpp>
#include <sim/vehicles/tinyquad.hpp>

namespace hf {

    class Simulator {

        public:

            void init(const bool tryJoystick=true)
            {
                wb_robot_init();

                _timestep = wb_robot_get_basic_time_step();

                if (tryJoystick) {

                    wb_joystick_enable(_timestep);
                }

                else {

                    printKeyboardInstructions();
                }

                wb_keyboard_enable(_timestep);

                _copter_node = wb_supervisor_node_get_from_def("ROBOT");

                _translation_field =
                    wb_supervisor_node_get_field(_copter_node, "translation");

                _rotation_field =
                    wb_supervisor_node_get_field(_copter_node, "rotation");

                _motor1 = make_motor("motor1");
                _motor2 = make_motor("motor2");
                _motor3 = make_motor("motor3");
                _motor4 = make_motor("motor4");

                // Start the dynamics thread
                _thread_data.running = true;
                _thread_data.dynamics = &_dynamics;
                pthread_create(
                        &_thread, NULL, *thread_fun, (void *)&_thread_data);
            }

            bool step(demands_t & open_loop_demands)
            {
                _thread_data.requested_takeoff = _requested_takeoff;

                memcpy(&_thread_data.open_loop_demands, &open_loop_demands,
                        sizeof(demands_t));

                if (wb_robot_step((int)_timestep) == -1) {

                    return false;
                } 

                auto posevals = _thread_data.posevals;

                const double pos[3] = {posevals[0], posevals[1], posevals[2]};
                wb_supervisor_field_set_sf_vec3f(_translation_field, pos);

                double rot[4] = {};
                angles_to_rotation(posevals[3], posevals[4], posevals[5], rot);
                wb_supervisor_field_set_sf_rotation(_rotation_field, rot);

                const auto motorvals = _thread_data.motorvals;

                // Negate expected direction to accommodate Webots
                // counterclockwise positive
                wb_motor_set_velocity(_motor1, -motorvals[0]);
                wb_motor_set_velocity(_motor2, +motorvals[1]);
                wb_motor_set_velocity(_motor3, +motorvals[2]);
                wb_motor_set_velocity(_motor4, -motorvals[3]);

                return true;
            }

            void close(void)
            {
                _thread_data.running = false;
                pthread_join(_thread, NULL);
            }

            state_t getState()
            {
                return _dynamics.state;
            }

            quad_motors_t getMotors() 
            {
                return quad_motors_t {
                    _thread_data.motorvals[0],
                    _thread_data.motorvals[1],
                    _thread_data.motorvals[2],
                    _thread_data.motorvals[3]
                };
            }

            bool requestedTakeoff()
            {
                return _requested_takeoff;
            }

            bool isSpringy()
            {
                return haveJoystick() == JOYSTICK_RECOGNIZED ?
                    getJoystickInfo().springy :
                    true; // keyboard
            }

            demands_t getDemands()
            {
                demands_t demands = {};

                auto joystickStatus = haveJoystick();

                if (joystickStatus == JOYSTICK_RECOGNIZED) {

                    auto axes = getJoystickInfo();

                    demands.thrust =
                        normalizeJoystickAxis(readJoystickRaw(axes.throttle));

                    // Springy throttle stick; keep in interval [-1,+1]
                    if (axes.springy) {

                        static bool button_was_hit;

                        if (wb_joystick_get_pressed_button() == 5) {
                            button_was_hit = true;
                        }

                        _requested_takeoff = button_was_hit;

                        // Run throttle stick through deadband
                        demands.thrust =
                            fabs(demands.thrust) < 0.05 ? 0 : demands.thrust;
                    }

                    else {

                        static float throttle_prev;
                        static bool throttle_was_moved;

                        // Handle bogus throttle values on startup
                        if (throttle_prev != demands.thrust) {
                            throttle_was_moved = true;
                        }

                        _requested_takeoff = throttle_was_moved;

                        throttle_prev = demands.thrust;
                    }

                    demands.roll = readJoystickAxis(axes.roll);
                    demands.pitch = readJoystickAxis(axes.pitch); 
                    demands.yaw = readJoystickAxis(axes.yaw);
                }

                else if (joystickStatus == JOYSTICK_UNRECOGNIZED) {

                    reportJoystick();
                }

                else { 

                    getDemandsFromKeyboard(demands);

                }

                return demands;
            }

 
        private:

            static constexpr float THRUST_BASE = 55.385;

            static const uint32_t DYNAMICS_FREQ = 10000;

            static const uint32_t PID_FREQ = 500;

            static constexpr float MOTOR_MAX = 60;

            typedef enum {

                JOYSTICK_NONE,
                JOYSTICK_UNRECOGNIZED,
                JOYSTICK_RECOGNIZED

            } joystickStatus_e;

            typedef struct {

                int8_t throttle;
                int8_t roll;
                int8_t pitch;
                int8_t yaw;

                bool springy;                

            } joystick_t;

            typedef struct {

                Dynamics * dynamics;
                bool requested_takeoff;
                demands_t open_loop_demands;
                float posevals[6];
                float motorvals[4];
                bool running;

            } thread_data_t;

             Dynamics _dynamics = Dynamics(tinyquad_params, 1.f/DYNAMICS_FREQ);

            bool _requested_takeoff;

            double _timestep;

            WbDeviceTag _motor1;
            WbDeviceTag _motor2;
            WbDeviceTag _motor3;
            WbDeviceTag _motor4;

            WbNodeRef _copter_node;
            WbFieldRef _translation_field;
            WbFieldRef _rotation_field;

            thread_data_t _thread_data; 
            pthread_t _thread; 

            static WbDeviceTag make_motor(const char * name)
            {
                auto motor = wb_robot_get_device(name);

                wb_motor_set_position(motor, INFINITY);

                return motor;
            }

            static float deg2rad(const float deg)
            {
                return M_PI * deg / 180;
            }

            static float max3(const float a, const float b, const float c)
            {
                return
                    a > b && a > c ? a :
                    b > a && b > c ? b :
                    c;
            }

            static float sign(const float val)
            {
                return val < 0 ? -1 : +1;
            }

            static float scale(const float angle, const float maxang)
            {
                return sign(angle) * sqrt(fabs(angle) / maxang);
            }

            static void angles_to_rotation(
                    const float phi, const float theta, const float psi,
                    double rs[4])
            {
                const auto phirad = deg2rad(phi);
                const auto therad = deg2rad(theta);
                const auto psirad = deg2rad(psi);

                const auto maxang =
                    max3(fabs(phirad), fabs(therad), fabs(psirad));

                if (maxang == 0) {
                    rs[0] = 0;
                    rs[1] = 0;
                    rs[2] = 1;
                    rs[3] = 0;
                }

                else {

                    rs[0] = scale(phi, maxang);
                    rs[1] = scale(theta, maxang);
                    rs[2] = scale(psi, maxang);
                    rs[3] = maxang;
                }
            }

            static float min(const float val, const float maxval)
            {
                return val > maxval ? maxval : val;
            }

            static void * thread_fun(void *ptr)
            {
                auto thread_data = (thread_data_t *)ptr;

                auto dynamics = thread_data->dynamics;

                demands_t demands = {};

                // We'll animate motors at full speed on startup, but won't run
                // dynamics
                quad_motors_t motors = {
                    MOTOR_MAX, MOTOR_MAX, MOTOR_MAX, MOTOR_MAX 
                };

                for (long k=0; thread_data->running; k++) {

                    if (thread_data->requested_takeoff) {

                        if (k % (DYNAMICS_FREQ / PID_FREQ) == 0) {

                            // Start with open-loop demands from main thread
                            memcpy(&demands, &thread_data->open_loop_demands,
                                    sizeof(demands_t));

                            extern void run_closed_loop_controllers(
                                    const float dt,
                                    const state_t & state, 
                                    demands_t & demands);

                            run_closed_loop_controllers(
                                    1.f/PID_FREQ, dynamics->state, demands);

                        }

                        const auto thrust =
                            min(demands.thrust + THRUST_BASE, MOTOR_MAX);

                        const demands_t new_demands = {
                            thrust, 0, 0, demands.yaw
                        };

                        hf::Mixer::runBetaFlightQuadX(new_demands, motors);

                        dynamics->setMotors(
                                motors.m1, 
                                motors.m2, 
                                motors.m3, 
                                motors.m4);

                        const auto state = dynamics->state;

                        thread_data->posevals[0] = state.x;
                        thread_data->posevals[1] = state.y;
                        thread_data->posevals[2] = state.z;
                        thread_data->posevals[3] = state.phi;
                        thread_data->posevals[4] = state.theta;
                        thread_data->posevals[5] = state.psi;
                    }

                    // Set motor spins for animation
                    thread_data->motorvals[0] = motors.m1;
                    thread_data->motorvals[1] = motors.m2;
                    thread_data->motorvals[2] = motors.m3;
                    thread_data->motorvals[3] = motors.m4;

                    // Throw in a delay to sync with the animation
                    usleep(1 / (DYNAMICS_FREQ * 1e-6));
                }

                return  ptr;
            }

            std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

                // Springy throttle
                { "MY-POWER CO.,LTD. 2In1 USB Joystick", // PS3
                    joystick_t {-2,  3, -4, 1, true } },
                { "SHANWAN Android Gamepad",             // PS3
                    joystick_t {-2,  3, -4, 1, true } },
                { "Logitech Gamepad F310",
                    joystick_t {-2,  4, -5, 1, true } },

                // Classic throttle
                { "Logitech Logitech Extreme 3D",
                    joystick_t {-4,  1, -2, 3, false}  },
                { "OpenTX FrSky Taranis Joystick",  // USB cable
                    joystick_t { 1,  2,  3, 4, false } },
                { "FrSky FrSky Simulator",          // radio dongle
                    joystick_t { 1,  2,  3, 4, false } },
                { "Horizon Hobby SPEKTRUM RECEIVER",
                    joystick_t { 2,  -3,  4, -1, false } }
            };

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

            joystick_t getJoystickInfo() 
            {
                return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
            }

            joystickStatus_e haveJoystick(void)
            {
                auto status = JOYSTICK_RECOGNIZED;

                auto joyname = wb_joystick_get_model();

                // No joystick
                if (joyname == NULL) {

                    static bool _didWarn;

                    if (!_didWarn) {
                        puts("Using keyboard instead:\n");
                        printKeyboardInstructions();
                    }

                    _didWarn = true;

                    status = JOYSTICK_NONE;
                }

                // Joystick unrecognized
                else if (JOYSTICK_AXIS_MAP.count(joyname) == 0) {

                    status = JOYSTICK_UNRECOGNIZED;
                }

                return status;
            }

            static void reportJoystick(void)
            {
                printf("Unrecognized joystick '%s' with axes ",
                        wb_joystick_get_model()); 

                for (uint8_t k=0; k<wb_joystick_get_number_of_axes(); ++k) {

                    printf("%2d=%+6d |", k+1, wb_joystick_get_axis_value(k));
                }
            }

            static void printKeyboardInstructions()
            {
                puts("- Use spacebar to take off\n");
                puts("- Use W and S to go up and down\n");
                puts("- Use arrow keys to move horizontally\n");
                puts("- Use Q and E to change heading\n");
            }

           void getDemandsFromKeyboard(demands_t & demands)
            {
                static bool spacebar_was_hit;

                switch (wb_keyboard_get_key()) {

                    case WB_KEYBOARD_UP:
                        demands.pitch = +1.0;
                        break;

                    case WB_KEYBOARD_DOWN:
                        demands.pitch = -1.0;
                        break;

                    case WB_KEYBOARD_RIGHT:
                        demands.roll = +1.0;
                        break;

                    case WB_KEYBOARD_LEFT:
                        demands.roll = -1.0;
                        break;

                    case 'Q':
                        demands.yaw = -1.0;
                        break;

                    case 'E':
                        demands.yaw = +1.0;
                        break;

                    case 'W':
                        demands.thrust = +1.0;
                        break;

                    case 'S':
                        demands.thrust = -1.0;
                        break;

                    case 32:
                        spacebar_was_hit = true;
                        break;
                }

                _requested_takeoff = spacebar_was_hit;
            }

    };

}
