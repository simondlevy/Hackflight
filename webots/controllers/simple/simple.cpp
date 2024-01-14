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

#include <stdio.h>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

// #include <simdvs.hpp>

#include <hackflight.hpp>
#include <mixers/quadrotor.hpp>

#include "sticks.hpp"

// These constants allow our PID constants to be in the same intervals as in
// the actual vehicle
static const float THRUST_BASE = 48;
static const float THRUST_SCALE = 0.25;
static const float THRUST_MIN = 0;
static const float THRUST_MAX   = 60;
static const float PITCH_ROLL_SCALE = 1e-4;
static const float YAW_SCALE = 4e-5;

static const Clock::rate_t PID_UPDATE_RATE = Clock::RATE_100_HZ;

// These constants set limits on the output of the altitude controller
static const float FLIGHT_CLIMB_RATE = 0.5;
static const float TAKEOFF_LAND_CLIMB_RATE= 0.1;

static WbDeviceTag makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

static void runCamera(WbDeviceTag &camera)
{
    /*
       auto image = Mat(Size(wb_camera_get_width(camera), 
       wb_camera_get_height(camera)), CV_8UC4);

       image.data = (uint8_t *)wb_camera_get_image(camera);

       auto events = dvs.getEvents(image);

       dvs.display(image, events);
     */
}

static vehicleState_t getVehicleState(
        WbDeviceTag & gyro, 
        WbDeviceTag & imu, 
        WbDeviceTag & gps)
{
    // Track previous time and position for calculating motion
    static float tprev;
    static float xprev;
    static float yprev;
    static float zprev;

    const auto tcurr = wb_robot_get_time();
    const auto dt =  tcurr - tprev;
    tprev = tcurr;

    // Get state values (meters, degrees) from ground truth:
    //   x: positive forward
    //   y: positive leftward
    //   z: positive upward
    //   phi, dphi: positive roll right
    //   theta,dtheta: positive nose up (requires negating imu, gyro)
    //   psi,dpsi: positive nose left
    const float x = wb_gps_get_values(gps)[0];
    const float y = wb_gps_get_values(gps)[1];
    const float z = wb_gps_get_values(gps)[2];
    const float phi =     rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[0]);
    const float dphi =    rad2deg(wb_gyro_get_values(gyro)[0]);
    const float theta =  -rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[1]);
    const float dtheta = -rad2deg(wb_gyro_get_values(gyro)[1]); 
    const float psi =     rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[2]);
    const float dpsi =    rad2deg(wb_gyro_get_values(gyro)[2]);

    // Use temporal first difference to get world-cooredinate velocities
    const float dx = (x - xprev) / dt;
    const float dy = (y - yprev) / dt;
    const float dz = (z - zprev) / dt;

    vehicleState_t state = {
        x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi
    };

    // Save past time and position for next time step
    xprev = x;
    yprev = y;
    zprev = z;

    return state;
}

static WbDeviceTag makeSensor(
        const char * name, 
        const uint32_t timestep,
        void (*f)(WbDeviceTag tag, int sampling_period))
{
    auto sensor = wb_robot_get_device(name);
    f(sensor, timestep);
    return sensor;
}

int main(int argc, char ** argv)
{
    static Hackflight hackflight;

    hackflight.init(
            mixQuadrotor,
            PID_UPDATE_RATE,
            THRUST_SCALE,
            THRUST_BASE,
            THRUST_MIN,
            THRUST_MAX,
            PITCH_ROLL_SCALE,
            YAW_SCALE);

    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors
    auto m1_motor = makeMotor("m1_motor", +1);
    auto m2_motor = makeMotor("m2_motor", -1);
    auto m3_motor = makeMotor("m3_motor", +1);
    auto m4_motor = makeMotor("m4_motor", -1);

    // Initialize sensors
    auto imu = makeSensor("inertial_unit", timestep, wb_inertial_unit_enable);
    auto gyro = makeSensor("gyro", timestep, wb_gyro_enable);
    auto gps = makeSensor("gps", timestep, wb_gps_enable);
    auto camera = makeSensor("camera", timestep, wb_camera_enable);

    // static SimDvs dvs;

    sticksInit();

    while (wb_robot_step(timestep) != -1) {

        runCamera(camera);

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        auto demands = sticksRead();

        // Check where we're in hover mode (button press on game controller)
        auto inHoverMode = sticksInHoverMode();

        // Altitude target, normalized to [-1,+1]
        static float _altitudeTarget;

        // Hover mode: integrate stick demand
        if (inHoverMode) {
            const float DT = .01;
            _altitudeTarget = Num::fconstrain(
                    _altitudeTarget + demands.thrust * DT, -1, +1);
            demands.thrust = _altitudeTarget;
        }

        // Non-hover mode: use raw stick value with min 0
        else {
            demands.thrust = Num::fconstrain(demands.thrust, 0, 1);
            _altitudeTarget = 0;
        }

        // Get vehicle state from sensors
        auto state = getVehicleState(gyro, imu, gps);

        // Run hackflight algorithm on open-loop demands and vehicle state to 
        // get motor values
        float motorvals[4] = {};
        hackflight.step(inHoverMode, state, demands, motorvals);

        // Set simulated motor values
        wb_motor_set_velocity(m1_motor, +motorvals[0]);
        wb_motor_set_velocity(m2_motor, -motorvals[1]);
        wb_motor_set_velocity(m3_motor, +motorvals[2]);
        wb_motor_set_velocity(m4_motor, -motorvals[3]);
    }

    wb_robot_cleanup();

    return 0;
}
