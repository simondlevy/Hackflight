/* 
 * Custom physics plugin for ping-pong autopilot simuation
 *
 *  Copyright (C) 2025 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// C
#include <stdlib.h>
#include <time.h>

// Hackflight
#include <sim/dynamics.hpp>
#include <plugin_helper.hpp>
#include "../autopilot.hpp"

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

// TennLab FPGS
#include <processor.hpp>

#define NUM_DECODERS (1)
#define NUM_OUTPUT_NEURONS (2)
#define NUM_ENCODERS (2)
#define TOT_MAX_ENCODED_SPIKES (100)

static const unsigned int SIM_TIME = 50;

typedef struct {
    int id;       /* Represents the input id of the destination neuron */
    double time;  /* Represents the timing of when the spike should arrive */
    double value; /* Represents the charge to accumulate */
} Spike;

void clear_encoded_spikes();
void decode();
void encode();

extern int decoder_counts[NUM_OUTPUT_NEURONS];
extern double encoder_vals[NUM_ENCODERS];
extern double decoder_vals[NUM_DECODERS];
extern unsigned int num_encoded_spikes;
extern Spike encoded_spikes[TOT_MAX_ENCODED_SPIKES]; 

static neuro::Processor proc_;

static constexpr float kSpeed = 0.5;

static AutopilotHelper * ahelper_;

static int readRangefinder(
        const string name,
        simsens::Robot & robot,
        simsens::World & world,
        const simsens::Pose & pose)
{
    auto rangefinder = robot.rangefinders[name];

    int distance_mm = 0;

    rangefinder.read(pose, world, &distance_mm);

    return distance_mm;
}

static auto getSetpoint(
        const int distance_forward_mm,
        const int distance_backward_mm,
        const float dydt) -> hf::Setpoint
{
    const auto diff = distance_forward_mm - distance_backward_mm;

    encoder_vals[0] = diff;
    encoder_vals[1] = dydt;

    clear_encoded_spikes();

    proc_.ClearActivity();

    encode();

    for (unsigned int i = 0; i < num_encoded_spikes; i++) {
        const auto spike = encoded_spikes[i];
        proc_.ApplySpike(spike.id, spike.time, spike.value);
    }
 
    proc_.Run(SIM_TIME);

    for (unsigned int i = 0; i < NUM_OUTPUT_NEURONS; i++) {
        decoder_counts[i] = proc_.GetOutputCount(i);
    }
    
    decode();

    printf("%+05.0f,%+6.6f => %2d %2d = > %1.0f\n",
            encoder_vals[0], encoder_vals[1],
            proc_.GetOutputCount(0), proc_.GetOutputCount(1),
            decoder_vals[0]);

    const int8_t direction = decoder_vals[0] == 1 ? +1 : -1;

    return hf::Setpoint(0, 0, direction * kSpeed, 0);
}

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    const auto message = PluginHelper::GetMessage();

    // Get current vehicle state
    const auto state = ahelper_->GetState(message);

    static int _distance_forward_mm;
    static int _distance_backward_mm;

    // Replace open-loop setpoint with setpoint from autopilot if
    // available
    const auto setpoint = message.mode == hf::kModeAutonomous ?
        getSetpoint(_distance_forward_mm, 
                _distance_backward_mm, state.dy) :
        message.setpoint;

    // Get vehicle pose based on setpoint
    const auto pose = ahelper_->GetPose(message.mode, setpoint);

    // Grab rangefinder readings for next iteration
    _distance_forward_mm = readRangefinder("VL53L1-forward",
            ahelper_->robot, ahelper_->world, pose);
    _distance_backward_mm = readRangefinder("VL53L1-backward",
            ahelper_->robot, ahelper_->world, pose);

    // Log data to file
    //const int distances[] = {_distance_forward_mm, _distance_backward_mm};
    //ahelper_->WriteToLog(pose, distances, 2);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete ahelper_;
}

DLLEXPORT void webots_physics_init() 
{
    srand(time(NULL)); 

    ahelper_ = new AutopilotHelper("pingpong");

    proc_.Connect();
}
