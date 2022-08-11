/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "datatypes.h"
#include "debug.h"
#include "maths.h"
#include "pids/angle.h"

// Datatypes ------------------------------------------------------------------

typedef struct {

    anglePid_t       anglepid;
    task_t           attitudeTask;
    demands_t        demands;
    gyro_t           gyro;
    imu_align_fun    imuAlignFun;
    imu_fusion_t     imuFusionPrev;
    float            maxArmingAngle;
    mixer_t          mixer;
    void *           motorDevice;
    float            mspMotors[4];
    task_t           mspTask;
    pid_controller_t pidControllers[10];
    uint8_t          pidCount;
    bool             pidReset;
    rx_t             rx;
    task_t           rxTask;
    rx_axes_t        rxAxes;
    scheduler_t      scheduler;
    vehicle_state_t  vstate;

} hackflight_t;

// Scheduling constants -------------------------------------------------------

static const uint32_t RX_TASK_RATE       = 33;
static const uint32_t ATTITUDE_TASK_RATE = 100;

// PID-limiting constants -----------------------------------------------------

static const float    PID_MIXER_SCALING = 1000;
static const uint16_t PIDSUM_LIMIT_YAW  = 400;
static const uint16_t PIDSUM_LIMIT      = 500;


// PID controller support -----------------------------------------------------

static void hackflightAddPidController(hackflight_t * hf, pid_fun_t fun, void * data)
{
    hf->pidControllers[hf->pidCount].fun = fun;
    hf->pidControllers[hf->pidCount].data = data;
    hf->pidCount++;
}

// Core tasks: gyro, PID controllers, mixer, motors ---------------------------

static float constrain_demand(float demand, float limit, float scaling)
{
    return constrain_f(demand, -limit, +limit) / scaling;
}

// Public API -----------------------------------------------------------------

static void hackflightRunCoreTasks(
        hackflight_t * hf,
        uint32_t usec,
        bool failsafe,
        motor_config_t * motorConfig,
        float motorvals[])
{
    // Run PID controllers to get new demands
    for (uint8_t k=0; k<hf->pidCount; ++k) {
        pid_controller_t pid = hf->pidControllers[k];
        pid.fun(usec, &hf->demands, pid.data, &hf->vstate, hf->pidReset);
    }


    // Constrain the demands, negating yaw to make it agree with PID
    demands_t * demands = &hf->demands;
    demands->roll  =
        constrain_demand(demands->roll, PIDSUM_LIMIT, PID_MIXER_SCALING);
    demands->pitch =
        constrain_demand(demands->pitch, PIDSUM_LIMIT, PID_MIXER_SCALING);
    demands->yaw   =
        -constrain_demand(demands->yaw, PIDSUM_LIMIT_YAW, PID_MIXER_SCALING);

    // Run the mixer to get motors from demands
    hf->mixer(&hf->demands, failsafe, motorConfig, motorvals);
}

static void hackflightInit(
        hackflight_t * hf,
        anglePidConstants_t * anglePidConstants,
        mixer_t mixer)
{
    hf->mixer = mixer;

    anglePidInit(&hf->anglepid, anglePidConstants);

    hackflightAddPidController(hf, anglePidUpdate, &hf->anglepid);
}

class HackflightCore {

    private:

        // PID-limiting constants

        static constexpr float    PID_MIXER_SCALING = 1000;
        static constexpr uint16_t PIDSUM_LIMIT_YAW  = 400;
        static constexpr uint16_t PIDSUM_LIMIT      = 500;

        // Instance variable

        anglePid_t       m_anglepid;
        demands_t        m_demands;
        mixer_t          m_mixer;
        pid_controller_t m_pidControllers[10];
        uint8_t          m_pidCount;
        bool             m_pidReset;

        static float constrain_demand(float demand, float limit, float scaling)
        {
            return constrain_f(demand, -limit, +limit) / scaling;
        }

    public:

        // Supports tasks
        vehicle_state_t  m_vstate;

        HackflightCore(anglePidConstants_t * anglePidConstants, mixer_t mixer)
        {
            m_mixer = mixer;

            anglePidInit(&m_anglepid, anglePidConstants);

            addPidController(anglePidUpdate, &m_anglepid);
        }

        void addPidController(pid_fun_t fun, void * data)
        {
            m_pidControllers[m_pidCount].fun = fun;
            m_pidControllers[m_pidCount].data = data;
            m_pidCount++;
        }

        void runCoreTasks(
                uint32_t usec,
                bool failsafe,
                motor_config_t * motorConfig,
                float motorvals[])
        {
            // Run PID controllers to get new demands
            for (uint8_t k=0; k<m_pidCount; ++k) {
                pid_controller_t pid = m_pidControllers[k];
                pid.fun(usec, &m_demands, pid.data, &m_vstate, m_pidReset);
            }

            // Constrain the demands, negating yaw to make it agree with PID
            demands_t * demands = &m_demands;
            demands->roll  =
                constrain_demand(demands->roll, PIDSUM_LIMIT, PID_MIXER_SCALING);
            demands->pitch =
                constrain_demand(demands->pitch, PIDSUM_LIMIT, PID_MIXER_SCALING);
            demands->yaw   =
                -constrain_demand(demands->yaw, PIDSUM_LIMIT_YAW, PID_MIXER_SCALING);

            // Run the mixer to get motors from demands
            m_mixer(&m_demands, failsafe, motorConfig, motorvals);
        }

}; // class Hackflight
