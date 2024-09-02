/*
  Pitch/roll angle PID-control algorithm for real and simulated flight
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
run reset kp ki ilimit dt target actual integ = (demand, integ') where

    error = target - actual

    demand = kp * error + ki * integ

    integ' = if reset then 0 else constrain (integ + error * dt) (-ilimit) ilimit

{--

  Demand is input as angles in degrees and output as angular velocities in
  degrees per second:

  roll: right-down positive

  pitch: nose-up positive

--}

pitchRollAnglePid reset dt state demands = demands' where

  kp = 6
  ki = 3
  ilimit = 20

  (rollDemand, rollInteg) = 
    run reset kp ki ilimit dt (roll demands) (phi state) rollInteg'

  (pitchDemand, pitchInteg) = 
    run reset kp ki ilimit dt (pitch demands) (theta state) pitchInteg'

  rollInteg' = [0] ++ rollInteg

  pitchInteg' = [0] ++ pitchInteg

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
*/
