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

static PhysicsPluginHelper _helper;

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    PhysicsPluginHelper::siminfo_t siminfo = {};

    if (_helper.get_siminfo(siminfo)) {

        const auto pose = _helper.get_pose_from_siminfo(siminfo);

        _helper.set_dbody_from_pose(pose);
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
}

DLLEXPORT void webots_physics_init() 
{
    _helper.init();
}
