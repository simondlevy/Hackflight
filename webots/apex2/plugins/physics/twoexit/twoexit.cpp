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

#include <sim/vehicles/quadx/apexquad.hpp>
#include "../helper.hpp"

static const uint8_t kRangefinderDisplayScaleup = 64;

static PluginHelper * helper_;

static auto getSetpoint(const int * rangefinder_distances_mm) -> hf::Setpoint
{
    const int * d = rangefinder_distances_mm;

    // Look for clear (infinity reading) in center of 1x8 readings
    const bool center_is_clear = d[3] == -1 && d[4] == -1;

    // If clear, pitch forward
    const auto pitch = center_is_clear ? 0.4 : 0;

    // Otherwise, yaw rightward
    const auto yaw = center_is_clear ? 0 : 0.2;

    return hf::Setpoint(0, 0, pitch, yaw);
}        


// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    static int _rangefinder_distances_mm[8];

    const auto message = PluginHelper::GetMessage();

    const auto vparams = hf::ApexQuad::kVehicleParams;

    const auto state = helper_->RunSimulator( hf::ApexQuad::Run, vparams,
            message.mode, message.setpoint);

    helper_->SetDbodyFromState(vparams, state);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete helper_;
}

DLLEXPORT void webots_physics_init() 
{
    helper_ = new PluginHelper();
}
