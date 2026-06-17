/* 
 * Custom physics plugin custom for Hackflight Webots-based simulator
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

#include "../helper.hpp"

static PluginHelper * _helper;

static dBodyID _rudder_left;
static dBodyID _rudder_right;

static void SetRudderDbody( dBodyID body, const hf::SimState & state,
        const float yaw)
{
    // Negate Y to make leftward positive
    dBodySetPosition(body, state.x, -state.y, state.z);

    const auto phi = state.phi - yaw;

    // Turn Euler angles into quaternion, negating psi for nose-left
    // positive
    const auto cr = (float)cos(phi / 2);
    const auto sr = (float)sin(phi / 2);
    const auto cp = (float)cos(state.theta / 2);
    const auto sp = (float)sin(state.theta / 2);
    const auto cy = (float)cos(-state.psi / 2);
    const auto sy = (float)sin(-state.psi / 2);

    const dQuaternion q = {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };

    dBodySetQuaternion(body, q);
}


DLLEXPORT void webots_physics_init() 
{
    _helper = new PluginHelper();

    _rudder_left = PluginHelper::InitBody("rudder_left");
    _rudder_right = PluginHelper::InitBody("rudder_right");
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    const auto message = PluginHelper::GetMessage();

    const auto state = _helper->RunSimulator(message.mode, message.setpoint);

    _helper->SetDbodyFromState(state);

    const auto yaw = _helper->GetSetpoint().yaw;

    SetRudderDbody(_rudder_left, state, yaw);
    SetRudderDbody(_rudder_right, state, yaw);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

