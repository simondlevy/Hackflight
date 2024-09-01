/*
  Pitch/roll angular rate PID-control algorithm for real and simulated flight
  controllers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
  */

/*
run nothrust reset kp ki kd ilimit dt target actual integ prev =
  (demand, integ', prev') where

    error = target - actual

    demand = if nothrust 
             then 0 else 
             kp * error + ki * integ + kd * (error - prev) / dt

    integ' = if reset then 0 
             else if nothrust then integ
             else constrain (integ + error * dt) (-ilimit) ilimit

    prev' = if reset then 0 
            else if nothrust then prev
            else error

{--

  Demands are input as angular velocities in degrees per second and output as
  as arbitrary values to be scaled according to motor characteristics:

  roll:  input roll-right positive => output positive

  pitch: input nose-up positive => output positive

--}

pitchRollRatePid reset dt state demands = demands' where

  kp = 125
  ki = 250
  kd = 1.25
  ilimit = 33

  nothrust = (thrust demands) == 0

  (rollDemand, rollInteg, rollPrev) = 
    run nothrust reset kp ki kd ilimit dt (roll demands) (dphi state)
        rollInteg' rollPrev'

  rollInteg' = [0] ++ rollInteg

  rollPrev' = [0] ++ rollPrev

  (pitchDemand, pitchInteg, pitchPrev) = 
    run nothrust reset kp ki kd ilimit dt (pitch demands) (dtheta state) 
        pitchInteg' pitchPrev'

  pitchInteg' = [0] ++ pitchInteg

  pitchPrev' = [0] ++ pitchPrev

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
  */
