/*
Copyright (c) 2022 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

/*
altHoldController kp ki (state, ready, demands) = (state, ready, demands')

  where

    demands' = Demands throttleDemand (roll demands) (pitch demands) (yaw demands)

    throttleDemand =  kp * err + ki * errI

    -- Compute error as altTarget velocity minus actual velocity, after
    -- negating actual to accommodate NED
    err = targetVelocity + (dz state)

    -- Accumualte error integral
    errI = constrain_abs (errI' + err) windupMax

    targetVelocity = if inband then altitudeTarget - altitude
                     else pilotVelZMax * (throttle demands)

    -- Reset controller when moving into deadband
    altitudeTarget = if inband && not (in_band throttleDemand' stickDeadband)
                     then altitude
                     else altitudeTarget'
    -- NED => ENU
    altitude = -(z state)

    -- inband = in_band throttleDemand stickDeadband
    inband = in_band (throttle demands) stickDeadband

    -- Controller state
    errI' = [0] ++ errI
    altitudeTarget' = [0] ++ altitudeTarget
    throttleDemand' = [0] ++ throttleDemand
    */
