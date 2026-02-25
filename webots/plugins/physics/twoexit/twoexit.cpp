/* 
 * Custom physics plugin for two-exit autopilot simuation
 *
 * Copyright (C) 2026 Simon D. Levy
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

// Hackflight
#include <datatypes.h>
#include "../helper.hpp"
#include "../autopilot.hpp"

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;
static constexpr float FRAME_RATE_HZ = 32;

static simsens::Rangefinder get_rangefinder(simsens::Robot & robot)
{
    return robot.rangefinders["VL53L5-forward"];
}

class TwoExitAutopilot {

    public:

        int rangefinder_distances_mm[8];

        static bool getSetpoint(TwoExitAutopilot & autopilot,
                const int frame, hf::Setpoint & setpoint)
        {
            static constexpr float TRAVEL_AFTER_CLEAR_SEC = 1;

            const int * d = autopilot.rangefinder_distances_mm;

            // Look for clear (infinity reading) in center of 1x8 readings
            const bool center_is_clear = d[3] == -1 && d[4] == -1;

            // If clear, pitch forward
            setpoint.pitch = center_is_clear ? 0.4 : 0;

            // Otherwise, yaw rightward
            setpoint.yaw = center_is_clear ? 0 : 0.2;

            // We're not done until all readings are infinity
            for (int i=0; i<8; ++i) {
                if (d[i] != -1) {
                    return false;
                }
            }

            static int _cleared_at_frame;

            if (_cleared_at_frame == 0) {
                _cleared_at_frame = frame;
            }

            // Travel a bit after exiting
            else if ((frame - _cleared_at_frame)/FRAME_RATE_HZ >
                    TRAVEL_AFTER_CLEAR_SEC) {
                return true;
            }

            return false;
        }        

        void readSensor(
                simsens::Robot & robot,
                simsens::World & world,
                const simsens::Pose & pose)
        {
            get_rangefinder(robot).read(pose, world, rangefinder_distances_mm);
        }
};

static TwoExitAutopilot _autopilot;

static AutopilotHelper * _helper;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    PluginHelper::siminfo_t siminfo = {};

    if (_helper->get_siminfo(siminfo)) {

        // Replace open-loop setpoint with setpoint from autopilot if
        // available
        if (siminfo.mode == hf::MODE_AUTONOMOUS) {
            static int _frame;
            TwoExitAutopilot::getSetpoint(_autopilot, _frame++, siminfo.setpoint);
        }

        // Get vehicle pose based on setpoint
        const auto pose = _helper->get_pose(siminfo);

        // Grab rangefinder distances for next iteration
        _autopilot.readSensor(_helper->robot, _helper->world, pose);

        // Log data to file
        _helper->write_to_log(
                pose, _autopilot.rangefinder_distances_mm, 8);

        // Display rangefinder distances
        simsens::RangefinderVisualizer::show(
                _autopilot.rangefinder_distances_mm,
                get_rangefinder(_helper->robot).min_distance_m,
                get_rangefinder(_helper->robot).max_distance_m,
                8, 1, RANGEFINDER_DISPLAY_SCALEUP);
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

DLLEXPORT void webots_physics_init() 
{
    _helper = new AutopilotHelper("twoexit");
}
