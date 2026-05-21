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

static FILE * _logfile;

static constexpr char PATH_VARIABLE_NAME[] = "WEBOTS_PATH";

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    const auto message = PluginHelper::get_message();

    const auto state = _helper->run_simulator(message.mode, message.setpoint);



    fprintf(_logfile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            dWebotsGetTime()/1000,
            state.dx, state.dy, state.z, state.dz, state.phi, state.dphi,
            state.theta, state.dtheta, state.psi, state.dpsi);

    _helper->set_dbody_from_state(state);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

DLLEXPORT void webots_physics_init() 
{
    const auto pwd = getenv(PATH_VARIABLE_NAME);

    char log_path[256] = {};
    sprintf(log_path, "%s/log.csv", pwd);

    _logfile = fopen(log_path, "w");

    fprintf(_logfile, "t,dx,dy,z,dz,phi,dphi,theta,dtheta,psi,dpsi\n");

    _helper = new PluginHelper();
}
