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
//static dBodyID _rudder_right;

static void SetRudderDbody( dBodyID body, const hf::SimState & state,
        const float xoff, const float yoff, const float zoff,
        const float yaw)
{
    // Negate Y to make leftward positive
    dBodySetPosition(body, state.x+xoff, -(state.y+yoff), state.z+zoff);

    const auto phi = state.phi - yaw;

    const auto theta = 0.f;
    const auto psi = 0.f;

    // Turn Euler angles into quaternion, negating psi for nose-left
    // positive
    const auto cr = (float)cos(phi / 2);
    const auto sr = (float)sin(phi / 2);
    const auto cp = (float)cos(theta / 2);
    const auto sp = (float)sin(theta / 2);
    const auto cy = (float)cos(-psi / 2);
    const auto sy = (float)sin(-psi / 2);

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
    //_rudder_right = PluginHelper::InitBody("rudder_right");
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    const auto message = PluginHelper::GetMessage();

    const auto state = _helper->RunSimulator(message.mode, message.setpoint);

    _helper->SetDbodyFromState(state, true);

    const auto yaw = _helper->GetSetpoint().yaw + M_PI;

    const float xoff = -0.250;
    const float yoff = -0.045;
    const float zoff = +0.044;

    SetRudderDbody(_rudder_left, state, xoff, yoff, zoff, yaw);
    //SetRudderDbody(_rudder_right, state, 0);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

