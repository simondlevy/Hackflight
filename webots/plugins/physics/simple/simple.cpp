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

#include "../common.hpp"

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    siminfo_t siminfo = {};

    if (PhysicsPluginHelper::get_siminfo(siminfo)) {

        const auto pose = PhysicsPluginHelper::get_pose_from_siminfo(siminfo);

        // Open log file first time through
        static FILE * _logfp;
        if (!_logfp) {
            _logfp = PhysicsPluginHelper::logfile_open(siminfo);
        }

        PhysicsPluginHelper::logfile_write_pose(_logfp, pose);

        fprintf(_logfp, "\n");
        fflush(_logfp);

        PhysicsPluginHelper::set_dbody_from_pose(pose);
    }
}
